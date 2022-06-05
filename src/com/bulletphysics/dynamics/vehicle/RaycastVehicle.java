package com.bulletphysics.dynamics.vehicle;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ContactConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.DoubleArrayList;

import java.util.ArrayList;

import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Raycast vehicle, very special constraint that turns a rigidbody into a vehicle.
 *
 * @author jezek2
 */
@SuppressWarnings("unused")
public class RaycastVehicle extends TypedConstraint {

    private final ArrayPool<double[]> doubleArrays = ArrayPool.get(double.class);

    private static final RigidBody fixedObject = new RigidBody(0, null, null);
    private static final double sideFrictionStiffness2 = 1.0;

    protected ArrayList<Vector3d> forwardWS = new ArrayList<>();
    protected ArrayList<Vector3d> axle = new ArrayList<>();
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

    public ArrayList<WheelInfo> wheelInfo = new ArrayList<>();

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
     * Basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed.
     */
    public WheelInfo addWheel(
            Vector3d connectionPointCS, Vector3d wheelDirectionCS0,
            Vector3d wheelAxleCS, double suspensionRestLength,
            double wheelRadius, VehicleTuning tuning,
            boolean isFrontWheel
    ) {
        WheelInfo ci = new WheelInfo();

        ci.chassisConnectionPointCS.set(connectionPointCS);
        ci.wheelDirectionCS.set(wheelDirectionCS0);
        ci.wheelAxleCS.set(wheelAxleCS);
        ci.suspensionRestLength = suspensionRestLength;
        ci.wheelRadius = wheelRadius;
        ci.suspensionStiffness = tuning.suspensionStiffness;
        ci.wheelDampingCompression = tuning.suspensionCompression;
        ci.wheelDampingRelaxation = tuning.suspensionDamping;
        ci.frictionSlip = tuning.frictionSlip;
        ci.bIsFrontWheel = isFrontWheel;
        ci.maxSuspensionTravelCm = tuning.maxSuspensionTravelCm;

        wheelInfo.add(ci);

        WheelInfo wheel = wheelInfo.get(getNumWheels() - 1);

        updateWheelTransformsWS(wheel, false);
        updateWheelTransform(getNumWheels() - 1, false);
        return wheel;
    }

    @SuppressWarnings("UnusedReturnValue")
    public Transform getWheelTransformWS(int wheelIndex, Transform out) {
        assert (wheelIndex < getNumWheels());
        WheelInfo wheel = wheelInfo.get(wheelIndex);
        out.set(wheel.worldTransform);
        return out;
    }

    public void updateWheelTransform(int wheelIndex) {
        updateWheelTransform(wheelIndex, true);
    }

    public void updateWheelTransform(int wheelIndex, boolean interpolatedTransform) {
        WheelInfo wheel = wheelInfo.get(wheelIndex);
        updateWheelTransformsWS(wheel, interpolatedTransform);
        Vector3d up = Stack.newVec();
        up.negate(wheel.raycastInfo.wheelDirectionWS);
        Vector3d right = wheel.raycastInfo.wheelAxleWS;
        Vector3d fwd = Stack.newVec();
        fwd.cross(up, right);
        fwd.normalize();
        // up = right.cross(fwd);
        // up.normalize();

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

        Stack.subVec(2);
        Stack.subMat(3);
        Stack.subQuat(2);

        wheel.worldTransform.origin.scaleAdd(wheel.raycastInfo.suspensionLength, wheel.raycastInfo.wheelDirectionWS, wheel.raycastInfo.hardPointWS);
    }

    public void resetSuspension() {
        int i;
        for (i = 0; i < wheelInfo.size(); i++) {
            WheelInfo wheel = wheelInfo.get(i);
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
    }

    @SuppressWarnings("UnusedReturnValue")
    public double rayCast(WheelInfo wheel) {
        updateWheelTransformsWS(wheel, false);

        double depth = -1.0;

        double rayLen = wheel.getSuspensionRestLength() + wheel.wheelRadius;

        Vector3d rayVector = Stack.newVec();
        rayVector.scale(rayLen, wheel.raycastInfo.wheelDirectionWS);
        Vector3d source = wheel.raycastInfo.hardPointWS;
        wheel.raycastInfo.contactPointWS.add(source, rayVector);
        Vector3d target = wheel.raycastInfo.contactPointWS;


        VehicleRaycasterResult rayResults = new VehicleRaycasterResult();

        assert (vehicleRaycaster != null);

        Object object = vehicleRaycaster.castRay(source, target, rayResults);

        wheel.raycastInfo.groundObject = null;

        if (object != null) {
            double param = rayResults.distFraction;
            depth = rayLen * rayResults.distFraction;
            wheel.raycastInfo.contactNormalWS.set(rayResults.hitNormalInWorld);
            wheel.raycastInfo.isInContact = true;

            wheel.raycastInfo.groundObject = fixedObject; // todo for driving on dynamic/movable objects!;
            //wheel.m_raycastInfo.m_groundObject = object;

            double hitDistance = param * rayLen;
            wheel.raycastInfo.suspensionLength = hitDistance - wheel.wheelRadius;
            // clamp on max suspension travel

            double minSuspensionLength = wheel.getSuspensionRestLength() - wheel.maxSuspensionTravelCm * 0.01;
            double maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.maxSuspensionTravelCm * 0.01;
            if (wheel.raycastInfo.suspensionLength < minSuspensionLength) {
                wheel.raycastInfo.suspensionLength = minSuspensionLength;
            }
            if (wheel.raycastInfo.suspensionLength > maxSuspensionLength) {
                wheel.raycastInfo.suspensionLength = maxSuspensionLength;
            }

            wheel.raycastInfo.contactPointWS.set(rayResults.hitPointInWorld);

            double denominator = wheel.raycastInfo.contactNormalWS.dot(wheel.raycastInfo.wheelDirectionWS);

            Vector3d chassis_velocity_at_contactPoint = Stack.newVec();
            Vector3d relpos = Stack.newVec();
            relpos.sub(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(Stack.newVec()));

            getRigidBody().getVelocityInLocalPoint(relpos, chassis_velocity_at_contactPoint);

            double projVel = wheel.raycastInfo.contactNormalWS.dot(chassis_velocity_at_contactPoint);

            if (denominator >= -0.1) {
                wheel.suspensionRelativeVelocity = 0;
                wheel.clippedInvContactDotSuspension = 1 / 0.1;
            } else {
                double inv = -1 / denominator;
                wheel.suspensionRelativeVelocity = projVel * inv;
                wheel.clippedInvContactDotSuspension = inv;
            }

            Stack.subVec(3);

        } else {
            // put wheel info as in rest position
            wheel.raycastInfo.suspensionLength = wheel.getSuspensionRestLength();
            wheel.suspensionRelativeVelocity = 0;
            wheel.raycastInfo.contactNormalWS.negate(wheel.raycastInfo.wheelDirectionWS);
            wheel.clippedInvContactDotSuspension = 1;
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

        currentVehicleSpeedKmHour = 3.6 * getRigidBody().getLinearVelocity(tmp).length();

        Transform chassisTrans = getChassisWorldTransform(Stack.newTrans());

        Vector3d forwardW = Stack.newVec();
        forwardW.set(
                chassisTrans.basis.getElement(0, indexForwardAxis),
                chassisTrans.basis.getElement(1, indexForwardAxis),
                chassisTrans.basis.getElement(2, indexForwardAxis));

        if (forwardW.dot(getRigidBody().getLinearVelocity(tmp)) < 0.0) {
            currentVehicleSpeedKmHour *= -1.0;
        }

        //
        // simulate suspension
        //


        for (WheelInfo info : wheelInfo) {
            rayCast(info);
        }

        updateSuspension(step);

        for (WheelInfo wheel : wheelInfo) {
            // apply suspension force
            double suspensionForce = wheel.wheelsSuspensionForce;

            double gMaxSuspensionForce = 6000f;
            if (suspensionForce > gMaxSuspensionForce) {
                suspensionForce = gMaxSuspensionForce;
            }
            Vector3d impulse = Stack.newVec();
            impulse.scale(suspensionForce * step, wheel.raycastInfo.contactNormalWS);
            Vector3d relpos = Stack.newVec();
            relpos.sub(wheel.raycastInfo.contactPointWS, getRigidBody().getCenterOfMassPosition(tmp));

            getRigidBody().applyImpulse(impulse, relpos);

            Stack.subVec(2);

        }

        updateFriction(step);

        for (WheelInfo wheel : wheelInfo) {
            Vector3d relPos = Stack.newVec();
            relPos.sub(wheel.raycastInfo.hardPointWS, getRigidBody().getCenterOfMassPosition(tmp));
            Vector3d vel = getRigidBody().getVelocityInLocalPoint(relPos, Stack.newVec());

            if (wheel.raycastInfo.isInContact) {
                Transform chassisWorldTransform = getChassisWorldTransform(Stack.newTrans());

                Vector3d fwd = Stack.newVec();
                fwd.set(
                        chassisWorldTransform.basis.getElement(0, indexForwardAxis),
                        chassisWorldTransform.basis.getElement(1, indexForwardAxis),
                        chassisWorldTransform.basis.getElement(2, indexForwardAxis)
                );

                double proj = fwd.dot(wheel.raycastInfo.contactNormalWS);
                tmp.scale(proj, wheel.raycastInfo.contactNormalWS);
                fwd.sub(tmp);

                double proj2 = fwd.dot(vel);

                wheel.deltaRotation = (proj2 * step) / (wheel.wheelRadius);
                wheel.rotation += wheel.deltaRotation;

                Stack.subVec(1); // fwd
                Stack.subTrans(1);

            } else {
                wheel.rotation += wheel.deltaRotation;
            }

            wheel.deltaRotation *= 0.99f; // damping of rotation when not in contact

            Stack.subVec(2);// relpos, vel

        }
    }

    public void setSteeringValue(double steering, int wheel) {
        assert (wheel >= 0 && wheel < getNumWheels());

        WheelInfo wheel_info = getWheelInfo(wheel);
        wheel_info.steering = steering;
    }

    public double getSteeringValue(int wheel) {
        return getWheelInfo(wheel).steering;
    }

    public void applyEngineForce(double force, int wheel) {
        assert (wheel >= 0 && wheel < getNumWheels());
        WheelInfo wheel_info = getWheelInfo(wheel);
        wheel_info.engineForce = force;
    }

    public WheelInfo getWheelInfo(int index) {
        assert ((index >= 0) && (index < getNumWheels()));

        return wheelInfo.get(index);
    }

    public void setBrake(double brake, int wheelIndex) {
        assert ((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
        getWheelInfo(wheelIndex).brake = brake;
    }

    public void updateSuspension(double deltaTime) {
        double chassisMass = 1.0 / chassisBody.getInvMass();

        for (int w_it = 0; w_it < getNumWheels(); w_it++) {
            WheelInfo wheelInfo = this.wheelInfo.get(w_it);

            if (wheelInfo.raycastInfo.isInContact) {
                double force;

                //Spring
                double suspensionLength = wheelInfo.getSuspensionRestLength();
                double currentLength = wheelInfo.raycastInfo.suspensionLength;
                double length_diff = (suspensionLength - currentLength);
                force = wheelInfo.suspensionStiffness * length_diff * wheelInfo.clippedInvContactDotSuspension;

                // Damper
                double projectedRelVel = wheelInfo.suspensionRelativeVelocity;
                double suspensionDamping;
                if (projectedRelVel < 0.0) {
                    suspensionDamping = wheelInfo.wheelDampingCompression;
                } else {
                    suspensionDamping = wheelInfo.wheelDampingRelaxation;
                }
                force -= suspensionDamping * projectedRelVel;

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

    private double calcRollingFriction(WheelContactPoint contactPoint) {
        Vector3d tmp = Stack.newVec();


        Vector3d contactPosWorld = contactPoint.frictionPositionWorld;

        Vector3d rel_pos1 = Stack.newVec();
        rel_pos1.sub(contactPosWorld, contactPoint.body0.getCenterOfMassPosition(tmp));
        Vector3d rel_pos2 = Stack.newVec();
        rel_pos2.sub(contactPosWorld, contactPoint.body1.getCenterOfMassPosition(tmp));

        double maxImpulse = contactPoint.maxImpulse;

        Vector3d vel1 = contactPoint.body0.getVelocityInLocalPoint(rel_pos1, Stack.newVec());
        Vector3d vel2 = contactPoint.body1.getVelocityInLocalPoint(rel_pos2, Stack.newVec());
        Vector3d vel = Stack.newVec();
        vel.sub(vel1, vel2);

        double vrel = contactPoint.frictionDirectionWorld.dot(vel);

        // calculate j that moves us to zero relative velocity
        double j1 = -vrel * contactPoint.jacDiagABInv;
        j1 = Math.min(j1, maxImpulse);
        j1 = Math.max(j1, -maxImpulse);

        return j1;
    }

    public void updateFriction(double timeStep) {
        // calculate the impulse, so that the wheels don't move sidewards
        int numWheel = getNumWheels();
        if (numWheel == 0) {
            return;
        }

        MiscUtil.resize(forwardWS, numWheel, Vector3d.class);
        MiscUtil.resize(axle, numWheel, Vector3d.class);
        MiscUtil.resize(forwardImpulse, numWheel);
        MiscUtil.resize(sideImpulse, numWheel);

        Vector3d tmp = Stack.newVec();

        int numWheelsOnGround = 0;

        // collapse all those loops into one!
        for (int i = 0; i < getNumWheels(); i++) {
            WheelInfo wheel_info = wheelInfo.get(i);
            RigidBody groundObject = (RigidBody) wheel_info.raycastInfo.groundObject;
            if (groundObject != null) {
                numWheelsOnGround++;
            }
            sideImpulse.set(i, 0.0);
            forwardImpulse.set(i, 0.0);
        }

        {
            Transform wheelTrans = Stack.newTrans();
            for (int i = 0; i < getNumWheels(); i++) {

                WheelInfo wheel_info = wheelInfo.get(i);

                RigidBody groundObject = (RigidBody) wheel_info.raycastInfo.groundObject;

                if (groundObject != null) {
                    getWheelTransformWS(i, wheelTrans);

                    Matrix3d wheelBasis0 = Stack.newMat(wheelTrans.basis);
                    axle.get(i).set(
                            wheelBasis0.getElement(0, indexRightAxis),
                            wheelBasis0.getElement(1, indexRightAxis),
                            wheelBasis0.getElement(2, indexRightAxis));

                    Vector3d surfNormalWS = wheel_info.raycastInfo.contactNormalWS;
                    double proj = axle.get(i).dot(surfNormalWS);
                    tmp.scale(proj, surfNormalWS);
                    axle.get(i).sub(tmp);
                    axle.get(i).normalize();

                    forwardWS.get(i).cross(surfNormalWS, axle.get(i));
                    forwardWS.get(i).normalize();

                    double[] doublePtr = doubleArrays.getFixed(1);
                    ContactConstraint.resolveSingleBilateral(chassisBody, wheel_info.raycastInfo.contactPointWS,
                            groundObject, wheel_info.raycastInfo.contactPointWS,
                            0.0, axle.get(i), doublePtr, timeStep);
                    sideImpulse.set(i, doublePtr[0]);
                    doubleArrays.release(doublePtr);

                    sideImpulse.set(i, sideImpulse.get(i) * sideFrictionStiffness2);
                }
            }
        }

        double sideFactor = 1.0;
        double fwdFactor = 0.5;

        boolean sliding = false;
        {
            for (int wheel = 0; wheel < getNumWheels(); wheel++) {
                WheelInfo wheel_info = wheelInfo.get(wheel);
                RigidBody groundObject = (RigidBody) wheel_info.raycastInfo.groundObject;

                double rollingFriction = 0.0;

                if (groundObject != null) {
                    if (wheel_info.engineForce != 0.0) {
                        rollingFriction = wheel_info.engineForce * timeStep;
                    } else {
                        double defaultRollingFrictionImpulse = 0.0;
                        double maxImpulse = wheel_info.brake != 0.0 ? wheel_info.brake : defaultRollingFrictionImpulse;
                        WheelContactPoint contactPt = new WheelContactPoint(chassisBody, groundObject, wheel_info.raycastInfo.contactPointWS, forwardWS.get(wheel), maxImpulse);
                        rollingFriction = calcRollingFriction(contactPt);
                    }
                }

                // switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

                forwardImpulse.set(wheel, 0.0);
                wheelInfo.get(wheel).skidInfo = 1.0;

                if (groundObject != null) {
                    wheelInfo.get(wheel).skidInfo = 1.0;

                    double maxImp = wheel_info.wheelsSuspensionForce * timeStep * wheel_info.frictionSlip;

                    double maxImpSquared = maxImp * maxImp;

                    forwardImpulse.set(wheel, rollingFriction); //wheelInfo.m_engineForce* timeStep;

                    double x = (forwardImpulse.get(wheel)) * fwdFactor;
                    double y = (sideImpulse.get(wheel)) * sideFactor;

                    double impulseSquared = (x * x + y * y);

                    if (impulseSquared > maxImpSquared) {
                        sliding = true;

                        double factor = maxImp / Math.sqrt(impulseSquared);

                        wheelInfo.get(wheel).skidInfo *= factor;
                    }
                }

            }
        }

        if (sliding) {
            for (int wheel = 0, l = getNumWheels(); wheel < l; wheel++) {
                if (sideImpulse.get(wheel) != 0.0) {
                    if (wheelInfo.get(wheel).skidInfo < 1.0) {
                        forwardImpulse.set(wheel, forwardImpulse.get(wheel) * wheelInfo.get(wheel).skidInfo);
                        sideImpulse.set(wheel, sideImpulse.get(wheel) * wheelInfo.get(wheel).skidInfo);
                    }
                }
            }
        }

        // apply the impulses
        {
            for (int wheel = 0, l = getNumWheels(); wheel < l; wheel++) {
                WheelInfo wheelInfo = this.wheelInfo.get(wheel);

                Vector3d relPos = Stack.newVec();
                relPos.sub(wheelInfo.raycastInfo.contactPointWS, chassisBody.getCenterOfMassPosition(tmp));

                if (forwardImpulse.get(wheel) != 0.0) {
                    tmp.scale(forwardImpulse.get(wheel), forwardWS.get(wheel));
                    chassisBody.applyImpulse(tmp, relPos);
                }
                if (sideImpulse.get(wheel) != 0.0) {
                    RigidBody groundObject = (RigidBody) this.wheelInfo.get(wheel).raycastInfo.groundObject;

                    Vector3d relPos2 = Stack.newVec();
                    relPos2.sub(wheelInfo.raycastInfo.contactPointWS, groundObject.getCenterOfMassPosition(tmp));

                    Vector3d sideImp = Stack.newVec();
                    sideImp.scale(sideImpulse.get(wheel), axle.get(wheel));

                    relPos.z *= wheelInfo.rollInfluence;
                    chassisBody.applyImpulse(sideImp, relPos);

                    // apply friction impulse on the ground
                    tmp.negate(sideImp);
                    groundObject.applyImpulse(tmp, relPos2);
                }
            }
        }
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
        Transform chassisTrans = getChassisWorldTransform(new Transform());

        out.set(
                chassisTrans.basis.getElement(0, indexForwardAxis),
                chassisTrans.basis.getElement(1, indexForwardAxis),
                chassisTrans.basis.getElement(2, indexForwardAxis));

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

    ////////////////////////////////////////////////////////////////////////////

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
