package com.bulletphysics.dynamics.vehicle;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ContactConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.DoubleArrayList;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Raycast vehicle, very special constraint that turn a rigidbody into a vehicle.
 *
 * @author jezek2
 */
@SuppressWarnings("unused")
public class RaycastVehicle extends TypedConstraint {

    private static final RigidBody FIXED_OBJECT = new RigidBody(0, null, null);
    private static final double sideFrictionStiffness2 = 1.0;

    protected ObjectArrayList<Vector3d> forwardWS = new ObjectArrayList<>();
    protected ObjectArrayList<Vector3d> axle = new ObjectArrayList<>();
    protected DoubleArrayList forwardImpulse = new DoubleArrayList();
    protected DoubleArrayList sideImpulse = new DoubleArrayList();

    private double tau;
    private double damping;
    private final VehicleRaycaster vehicleRaycaster;
    private double pitchControl = 0.0;
    private double steeringValue;
    private double currentVehicleSpeedKmHour;

    private final RigidBody chassisBody;

    private int indexRightAxis = 0;
    private int indexUpAxis = 2;
    private int indexForwardAxis = 1;

    public ObjectArrayList<WheelInfo> wheelInfo = new ObjectArrayList<>();

    // constructor to create a car from an existing rigidbody
    public RaycastVehicle(VehicleTuning tuning, RigidBody chassis, VehicleRaycaster raycaster) {
        super(TypedConstraintType.VEHICLE_CONSTRAINT_TYPE);
        this.vehicleRaycaster = raycaster;
        this.chassisBody = chassis;
        defaultInit(tuning);
    }

    private void defaultInit(VehicleTuning tuning) {
        currentVehicleSpeedKmHour = 0.0;
        steeringValue = 0.0;
    }

    /**
     * Basically most of the code is general for 2 or 4-wheel vehicles, but some of it needs to be reviewed.
     */
    public WheelInfo addWheel(
            Vector3d connectionPointCS, Vector3d wheelDirectionCS0, Vector3d wheelAxleCS,
            double suspensionRestLength, double wheelRadius, VehicleTuning tuning, boolean isFrontWheel) {
        WheelInfoConstructionInfo ci = new WheelInfoConstructionInfo();

        ci.chassisConnectionCS.set(connectionPointCS);
        ci.wheelDirectionCS.set(wheelDirectionCS0);
        ci.wheelAxleCS.set(wheelAxleCS);
        ci.suspensionRestLength = suspensionRestLength;
        ci.wheelRadius = wheelRadius;
        ci.suspensionStiffness = tuning.suspensionStiffness;
        ci.wheelsDampingCompression = tuning.suspensionCompression;
        ci.wheelsDampingRelaxation = tuning.suspensionDamping;
        ci.frictionSlip = tuning.frictionSlip;
        ci.bIsFrontWheel = isFrontWheel;
        ci.maxSuspensionTravelCm = tuning.maxSuspensionTravelCm;

        wheelInfo.add(new WheelInfo(ci));

        WheelInfo wheel = wheelInfo.getQuick(getNumWheels() - 1);

        updateWheelTransformsWS(wheel, false);
        updateWheelTransform(getNumWheels() - 1, false);
        return wheel;
    }

    @SuppressWarnings("UnusedReturnValue")
    public Transform getWheelTransformWS(int wheelIndex, Transform out) {
        assert (wheelIndex < getNumWheels());
        WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
        out.set(wheel.worldTransform);
        return out;
    }

    public void updateWheelTransform(int wheelIndex) {
        updateWheelTransform(wheelIndex, true);
    }

    public void updateWheelTransform(int wheelIndex, boolean interpolatedTransform) {
        WheelInfo wheel = wheelInfo.getQuick(wheelIndex);
        updateWheelTransformsWS(wheel, interpolatedTransform);
        Vector3d up = Stack.newVec();
        up.negate(wheel.raycastInfo.wheelDirectionWS);
        Vector3d right = wheel.raycastInfo.wheelAxleWS;
        Vector3d fwd = Stack.newVec();
        fwd.cross(up, right);
        fwd.normalize();

        // rotate around steering over de wheelAxleWS
        double steering = wheel.steering;

        Quat4d steeringOrn = Stack.newQuat();
        QuaternionUtil.setRotation(steeringOrn, up, steering); //wheel.m_steering);
        Matrix3d steeringMat = Stack.newMat();
        MatrixUtil.setRotation(steeringMat, steeringOrn);

        Quat4d rotatingOrn = Stack.newQuat();
        QuaternionUtil.setRotation(rotatingOrn, right, -wheel.rotation);
        Matrix3d rotatingMat = Stack.newMat();
        MatrixUtil.setRotation(rotatingMat, rotatingOrn);

        Matrix3d basis2 = Stack.newMat();
        basis2.setRow(0, right.x, fwd.x, up.x);
        basis2.setRow(1, right.y, fwd.y, up.y);
        basis2.setRow(2, right.z, fwd.z, up.z);

        Matrix3d wheelBasis = wheel.worldTransform.basis;
        wheelBasis.mul(steeringMat, rotatingMat);
        wheelBasis.mul(basis2);

        wheel.worldTransform.origin.scaleAdd(wheel.raycastInfo.suspensionLength, wheel.raycastInfo.wheelDirectionWS, wheel.raycastInfo.hardPointWS);

        Stack.subVec(2);
        Stack.subMat(3);
        Stack.subQuat(2);
    }

    public void resetSuspension() {
        int i;
        for (i = 0; i < wheelInfo.size(); i++) {
            WheelInfo wheel = wheelInfo.getQuick(i);
            wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
            wheel.suspensionRelativeVelocity = 0.0;

            wheel.raycastInfo.contactNormalWS.negate(wheel.raycastInfo.wheelDirectionWS);
            //wheel_info.setContactFriction(btScalar(0.0));
            wheel.clippedInvContactDotSuspension = 1.0;
        }
    }

    public void updateWheelTransformsWS(WheelInfo wheel) {
        updateWheelTransformsWS(wheel, true);
    }

    public void updateWheelTransformsWS(WheelInfo wheel, boolean interpolatedTransform) {
        wheel.raycastInfo.isInContact = false;

        Transform chassisTrans = getChassisWorldTransform(Stack.newTrans());
        if (interpolatedTransform && (getRigidBody().getMotionState() != null)) {
            getRigidBody().getMotionState().getWorldTransform(chassisTrans);
        }

        wheel.raycastInfo.hardPointWS.set(wheel.chassisConnectionPointCS);
        chassisTrans.transform(wheel.raycastInfo.hardPointWS);

        wheel.raycastInfo.wheelDirectionWS.set(wheel.wheelDirectionCS);
        chassisTrans.basis.transform(wheel.raycastInfo.wheelDirectionWS);

        wheel.raycastInfo.wheelAxleWS.set(wheel.wheelAxleCS);
        chassisTrans.basis.transform(wheel.raycastInfo.wheelAxleWS);
        Stack.subTrans(1);
    }

    @SuppressWarnings("UnusedReturnValue")
    public double rayCast(WheelInfo wheel) {
        updateWheelTransformsWS(wheel, false);

        double depth = -1.0;

        double rayLength = wheel.getSuspensionRestLength() + wheel.wheelRadius;

        Vector3d rayVector = Stack.newVec();
        rayVector.scale(rayLength, wheel.raycastInfo.wheelDirectionWS);
        Vector3d source = wheel.raycastInfo.hardPointWS;
        wheel.raycastInfo.contactPointWS.add(source, rayVector);
        Vector3d target = wheel.raycastInfo.contactPointWS;

        double param;

        VehicleRaycasterResult rayResults = new VehicleRaycasterResult();

        assert (vehicleRaycaster != null);

        Object object = vehicleRaycaster.castRay(source, target, rayResults);

        wheel.raycastInfo.groundObject = null;

        if (object != null) {
            param = rayResults.distFraction;
            depth = rayLength * rayResults.distFraction;
            wheel.raycastInfo.contactNormalWS.set(rayResults.hitNormalInWorld);
            wheel.raycastInfo.isInContact = true;

            wheel.raycastInfo.groundObject = FIXED_OBJECT; // todo for driving on dynamic/movable objects!;
            //wheel.m_raycastInfo.m_groundObject = object;

            double hitDistance = param * rayLength;
            wheel.raycastInfo.suspensionLength = hitDistance - wheel.wheelRadius;
            // clamp on max suspension travel

            double minSuspensionLength = wheel.getSuspensionRestLength() - wheel.maxSuspensionTravelCm * 0.01f;
            double maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.maxSuspensionTravelCm * 0.01f;
            if (wheel.raycastInfo.suspensionLength < minSuspensionLength) {
                wheel.raycastInfo.suspensionLength = minSuspensionLength;
            }
            if (wheel.raycastInfo.suspensionLength > maxSuspensionLength) {
                wheel.raycastInfo.suspensionLength = maxSuspensionLength;
            }

            wheel.raycastInfo.contactPointWS.set(rayResults.hitPointInWorld);

            double denominator = wheel.raycastInfo.contactNormalWS.dot(wheel.raycastInfo.wheelDirectionWS);

            Vector3d chassisVelocityAtContactPoint = Stack.newVec();
            Vector3d relativePosition = Stack.newVec();
            relativePosition.sub(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(Stack.newVec()));

            getRigidBody().getVelocityInLocalPoint(relativePosition, chassisVelocityAtContactPoint);

            double projVel = wheel.raycastInfo.contactNormalWS.dot(chassisVelocityAtContactPoint);

            if (denominator >= -0.1) {
                wheel.suspensionRelativeVelocity = 0.0;
                wheel.clippedInvContactDotSuspension = 1.0 / 0.1;
            } else {
                double inv = -1.0 / denominator;
                wheel.suspensionRelativeVelocity = projVel * inv;
                wheel.clippedInvContactDotSuspension = inv;
            }

        } else {
            // put wheel info as in rest position
            wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
            wheel.suspensionRelativeVelocity = 0.0;
            wheel.raycastInfo.contactNormalWS.negate(wheel.raycastInfo.wheelDirectionWS);
            wheel.clippedInvContactDotSuspension = 1.0;
        }

        return depth;
    }

    public Transform getChassisWorldTransform(Transform out) {
		/*
		if (getRigidBody()->getMotionState())
		{
			btTransform chassisWorldTrans;
			getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
			return chassisWorldTrans;
		}
		*/

        return getRigidBody().getCenterOfMassTransform(out);
    }

    public void updateVehicle(double step) {
        for (int i = 0; i < getNumWheels(); i++) {
            updateWheelTransform(i, false);
        }

        Vector3d tmp = Stack.newVec();

        currentVehicleSpeedKmHour = 3.6f * getRigidBody().getLinearVelocity(tmp).length();

        Vector3d forwardW = Stack.newVec();
        Transform chassisTrans = getChassisWorldTransform(Stack.newTrans());
        forwardW.set(
                chassisTrans.basis.getElement(0, indexForwardAxis),
                chassisTrans.basis.getElement(1, indexForwardAxis),
                chassisTrans.basis.getElement(2, indexForwardAxis));
        Stack.subTrans(1); // chassisTrans

        if (forwardW.dot(getRigidBody().getLinearVelocity(tmp)) < 0.0) {
            currentVehicleSpeedKmHour *= -1.0;
        }

        //
        // simulate suspension
        //

        for (int i = 0; i < wheelInfo.size(); i++) {
            rayCast(wheelInfo.getQuick(i));
        }

        updateSuspension(step);

        for (int i = 0; i < wheelInfo.size(); i++) {
            // apply suspension force
            WheelInfo wheel = wheelInfo.getQuick(i);

            double suspensionForce = wheel.wheelsSuspensionForce;

            double gMaxSuspensionForce = 6000f;
            if (suspensionForce > gMaxSuspensionForce) {
                suspensionForce = gMaxSuspensionForce;
            }
            Vector3d impulse = Stack.newVec();
            impulse.scale(suspensionForce * step, wheel.raycastInfo.contactNormalWS);
            Vector3d relPos = Stack.newVec();
            relPos.sub(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(tmp));

            getRigidBody().applyImpulse(impulse, relPos);
            Stack.subVec(2);
        }

        updateFriction(step);

        Vector3d relPos = Stack.newVec();
        Vector3d vel = Stack.newVec();
        for (int i = 0; i < wheelInfo.size(); i++) {
            WheelInfo wheel = wheelInfo.getQuick(i);
            relPos.sub(wheel.raycastInfo.hardPointWS, getRigidBody().getCenterOfMassPosition(tmp));
            getRigidBody().getVelocityInLocalPoint(relPos, vel);

            if (wheel.raycastInfo.isInContact) {
                Transform chassisWorldTransform = getChassisWorldTransform(Stack.newTrans());

                Vector3d fwd = Stack.newVec();
                fwd.set(
                        chassisWorldTransform.basis.getElement(0, indexForwardAxis),
                        chassisWorldTransform.basis.getElement(1, indexForwardAxis),
                        chassisWorldTransform.basis.getElement(2, indexForwardAxis));

                double proj = fwd.dot(wheel.raycastInfo.contactNormalWS);
                tmp.scale(proj, wheel.raycastInfo.contactNormalWS);
                fwd.sub(tmp);

                double proj2 = fwd.dot(vel);

                wheel.deltaRotation = (proj2 * step) / (wheel.wheelRadius);
                Stack.subVec(1);
                Stack.subTrans(1);
            }

            wheel.rotation += wheel.deltaRotation;
            wheel.deltaRotation *= 0.99f; // damping of rotation when not in contact
        }
        Stack.subVec(4);
    }

    public void setSteeringValue(double steering, int wheel) {
        assert (wheel >= 0 && wheel < getNumWheels());

        WheelInfo wheelInfo = getWheelInfo(wheel);
        wheelInfo.steering = steering;
    }

    public double getSteeringValue(int wheel) {
        return getWheelInfo(wheel).steering;
    }

    public void applyEngineForce(double force, int wheel) {
        assert (wheel >= 0 && wheel < getNumWheels());
        WheelInfo wheelInfo = getWheelInfo(wheel);
        wheelInfo.engineForce = force;
    }

    public WheelInfo getWheelInfo(int index) {
        assert ((index >= 0) && (index < getNumWheels()));

        return wheelInfo.getQuick(index);
    }

    public void setBrake(double brake, int wheelIndex) {
        assert ((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
        getWheelInfo(wheelIndex).brake = brake;
    }

    public void updateSuspension(double deltaTime) {
        double chassisMass = 1.0 / chassisBody.getInvMass();

        for (int wheelIndex = 0; wheelIndex < getNumWheels(); wheelIndex++) {
            WheelInfo wheelInfo = this.wheelInfo.getQuick(wheelIndex);

            if (wheelInfo.raycastInfo.isInContact) {
                double force;
                //	Spring
                {
                    double susp_length = wheelInfo.getSuspensionRestLength();
                    double current_length = wheelInfo.raycastInfo.suspensionLength;

                    double length_diff = (susp_length - current_length);

                    force = wheelInfo.suspensionStiffness * length_diff * wheelInfo.clippedInvContactDotSuspension;
                }

                // Damper
                {
                    double projectedRelVel = wheelInfo.suspensionRelativeVelocity;
                    {
                        double suspensionDamping;
                        if (projectedRelVel < 0.0) {
                            suspensionDamping = wheelInfo.wheelDampingCompression;
                        } else {
                            suspensionDamping = wheelInfo.wheelDampingRelaxation;
                        }
                        force -= suspensionDamping * projectedRelVel;
                    }
                }

                // RESULT
                wheelInfo.wheelsSuspensionForce = force * chassisMass;
                if (wheelInfo.wheelsSuspensionForce < 0.0) {
                    wheelInfo.wheelsSuspensionForce = 0.0;
                }
            } else {
                wheelInfo.wheelsSuspensionForce = 0.0;
            }
        }
    }

    private double calcRollingFriction(WheelContactPoint contactPoint, int numWheelsOnGround) {
        Vector3d tmp = Stack.newVec();

        Vector3d contactPosWorld = contactPoint.frictionPositionWorld;

        Vector3d relPos1 = Stack.newVec();
        relPos1.sub(contactPosWorld, contactPoint.body0.getCenterOfMassPosition(tmp));
        Vector3d relPos2 = Stack.newVec();
        relPos2.sub(contactPosWorld, contactPoint.body1.getCenterOfMassPosition(tmp));

        double maxImpulse = contactPoint.maxImpulse;

        Vector3d vel1 = contactPoint.body0.getVelocityInLocalPoint(relPos1, Stack.newVec());
        Vector3d vel2 = contactPoint.body1.getVelocityInLocalPoint(relPos2, Stack.newVec());
        Vector3d vel = Stack.newVec();
        vel.sub(vel1, vel2);

        double relativeVelocity = contactPoint.frictionDirectionWorld.dot(vel);

        // calculate j that moves us to zero relative velocity
        double impulse = -relativeVelocity * contactPoint.jacDiagABInv / numWheelsOnGround;
        impulse = Math.min(impulse, maxImpulse);
        impulse = Math.max(impulse, -maxImpulse);

        Stack.subVec(6);

        return impulse;
    }

    public void updateFriction(double timeStep) {
        // calculate the impulse, so that the wheels don't move sidewards
        int numWheel = getNumWheels();
        if (numWheel == 0) {
            return;
        }

        MiscUtil.resize(forwardWS, numWheel, Vector3d.class);
        MiscUtil.resize(axle, numWheel, Vector3d.class);
        MiscUtil.resize(forwardImpulse, numWheel, 0.0);
        MiscUtil.resize(sideImpulse, numWheel, 0.0);

        Vector3d tmp = Stack.newVec();

        int numWheelsOnGround = 0;

        // collapse all those loops into one!
        for (int i = 0; i < getNumWheels(); i++) {
            WheelInfo wheelInfo = this.wheelInfo.getQuick(i);
            RigidBody groundObject = wheelInfo.raycastInfo.groundObject;
            if (groundObject != null) {
                numWheelsOnGround++;
            }
            sideImpulse.set(i, 0.0);
            forwardImpulse.set(i, 0.0);
        }

        {
            Transform wheelTrans = Stack.newTrans();
            double[] impulse = Stack.newDoublePtr();
            for (int i = 0; i < getNumWheels(); i++) {

                WheelInfo wheelInfo = this.wheelInfo.getQuick(i);
                RigidBody groundObject = wheelInfo.raycastInfo.groundObject;

                if (groundObject != null) {
                    getWheelTransformWS(i, wheelTrans);

                    axle.getQuick(i).set(
                            wheelTrans.basis.getElement(0, indexRightAxis),
                            wheelTrans.basis.getElement(1, indexRightAxis),
                            wheelTrans.basis.getElement(2, indexRightAxis));

                    Vector3d surfNormalWS = wheelInfo.raycastInfo.contactNormalWS;
                    double proj = axle.getQuick(i).dot(surfNormalWS);
                    tmp.scale(proj, surfNormalWS);
                    axle.getQuick(i).sub(tmp);
                    axle.getQuick(i).normalize();

                    forwardWS.getQuick(i).cross(surfNormalWS, axle.getQuick(i));
                    forwardWS.getQuick(i).normalize();

                    ContactConstraint.resolveSingleBilateral(chassisBody, wheelInfo.raycastInfo.contactPointWS,
                            groundObject, wheelInfo.raycastInfo.contactPointWS,
                            0f, axle.getQuick(i), impulse, timeStep);
                    sideImpulse.set(i, impulse[0]);
                    sideImpulse.set(i, sideImpulse.get(i) * sideFrictionStiffness2);
                }
            }
            Stack.subTrans(1);
            Stack.subDoublePtr(1);
        }

        double sideFactor = 1.0;
        double fwdFactor = 0.5;

        boolean sliding = false;
        for (int wheel = 0; wheel < getNumWheels(); wheel++) {
            WheelInfo wheelInfo = this.wheelInfo.getQuick(wheel);
            RigidBody groundObject = wheelInfo.raycastInfo.groundObject;

            double rollingFriction = 0.0;

            if (groundObject != null) {
                if (wheelInfo.engineForce != 0.0) {
                    rollingFriction = wheelInfo.engineForce * timeStep;
                } else {
                    double defaultRollingFrictionImpulse = 0.0;
                    double maxImpulse = wheelInfo.brake != 0.0 ? wheelInfo.brake : defaultRollingFrictionImpulse;
                    WheelContactPoint contactPt = new WheelContactPoint(
                            chassisBody, groundObject, wheelInfo.raycastInfo.contactPointWS,
                            forwardWS.getQuick(wheel), maxImpulse);
                    rollingFriction = calcRollingFriction(contactPt, numWheelsOnGround);
                }
            }

            // switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

            forwardImpulse.set(wheel, 0.0);
            this.wheelInfo.getQuick(wheel).skidInfo = 1.0;

            if (groundObject != null) {
                this.wheelInfo.getQuick(wheel).skidInfo = 1.0;

                double maxImpulse = wheelInfo.wheelsSuspensionForce * timeStep * wheelInfo.frictionSlip;

                forwardImpulse.set(wheel, rollingFriction); //wheelInfo.m_engineForce* timeStep;

                double x = (forwardImpulse.get(wheel)) * fwdFactor;
                double y = (sideImpulse.get(wheel)) * sideFactor;

                double impulseSquared = (x * x + y * y);
                if (impulseSquared > maxImpulse * maxImpulse) {
                    sliding = true;

                    double factor = maxImpulse / Math.sqrt(impulseSquared);
                    this.wheelInfo.getQuick(wheel).skidInfo *= factor;
                }
            }
        }

        if (sliding) {
            for (int wheel = 0; wheel < getNumWheels(); wheel++) {
                if (sideImpulse.get(wheel) != 0.0) {
                    if (wheelInfo.getQuick(wheel).skidInfo < 1.0) {
                        forwardImpulse.set(wheel, forwardImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
                        sideImpulse.set(wheel, sideImpulse.get(wheel) * wheelInfo.getQuick(wheel).skidInfo);
                    }
                }
            }
        }

        // apply the impulses
        {
            Vector3d relPos = Stack.newVec();
            for (int wheel = 0; wheel < getNumWheels(); wheel++) {
                WheelInfo wheelInfo = this.wheelInfo.getQuick(wheel);
                relPos.sub(wheelInfo.raycastInfo.contactPointWS, chassisBody.getCenterOfMassPosition(tmp));

                if (forwardImpulse.get(wheel) != 0.0) {
                    tmp.scale(forwardImpulse.get(wheel), forwardWS.getQuick(wheel));
                    chassisBody.applyImpulse(tmp, relPos);
                }
                if (sideImpulse.get(wheel) != 0.0) {
                    RigidBody groundObject = this.wheelInfo.getQuick(wheel).raycastInfo.groundObject;

                    Vector3d relPos2 = Stack.newVec();
                    relPos2.sub(wheelInfo.raycastInfo.contactPointWS, groundObject.getCenterOfMassPosition(tmp));

                    Vector3d sideImp = Stack.newVec();
                    sideImp.scale(sideImpulse.get(wheel), axle.getQuick(wheel));

                    relPos.z *= wheelInfo.rollInfluence;
                    chassisBody.applyImpulse(sideImp, relPos);

                    // apply friction impulse on the ground
                    tmp.negate(sideImp);
                    groundObject.applyImpulse(tmp, relPos2);

                    Stack.subVec(2);
                }
            }
            Stack.subVec(1); // relPos
        }
        Stack.subVec(1); // tmp
    }

    @Override
    public void buildJacobian() {
        // not yet
    }

    @Override
    public void solveConstraint(double timeStep) {
        // not yet
    }

    public int getNumWheels() {
        return wheelInfo.size();
    }

    public void setPitchControl(double pitch) {
        this.pitchControl = pitch;
    }

    public RigidBody getRigidBody() {
        return chassisBody;
    }

    public int getRightAxis() {
        return indexRightAxis;
    }

    public int getUpAxis() {
        return indexUpAxis;
    }

    public int getForwardAxis() {
        return indexForwardAxis;
    }

    /**
     * Worldspace forward vector.
     */
    public Vector3d getForwardVector(Vector3d out) {
        Transform chassisTrans = getChassisWorldTransform(Stack.newTrans());
        out.set(
                chassisTrans.basis.getElement(0, indexForwardAxis),
                chassisTrans.basis.getElement(1, indexForwardAxis),
                chassisTrans.basis.getElement(2, indexForwardAxis));
        Stack.subTrans(1);
        return out;
    }

    /**
     * Velocity of vehicle (positive if velocity vector has same direction as foward vector).
     */
    public double getCurrentSpeedKmHour() {
        return currentVehicleSpeedKmHour;
    }

    public void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
        this.indexRightAxis = rightIndex;
        this.indexUpAxis = upIndex;
        this.indexForwardAxis = forwardIndex;
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static class WheelContactPoint {
        public RigidBody body0;
        public RigidBody body1;
        public final Vector3d frictionPositionWorld = new Vector3d();
        public final Vector3d frictionDirectionWorld = new Vector3d();
        public double jacDiagABInv;
        public double maxImpulse;

        public WheelContactPoint(RigidBody body0, RigidBody body1, Vector3d frictionPosWorld, Vector3d frictionDirectionWorld, double maxImpulse) {
            this.body0 = body0;
            this.body1 = body1;
            this.frictionPositionWorld.set(frictionPosWorld);
            this.frictionDirectionWorld.set(frictionDirectionWorld);
            this.maxImpulse = maxImpulse;

            double denom0 = body0.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
            double denom1 = body1.computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
            double relaxation = 1.0;
            jacDiagABInv = relaxation / (denom0 + denom1);
        }
    }

}
