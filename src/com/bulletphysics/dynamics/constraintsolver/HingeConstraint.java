
/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */
package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Hinge constraint between two rigid bodies each with a pivot point that descibes
 * the axis location in local space. Axis defines the orientation of the hinge axis.
 *
 * @author jezek2
 */
public class HingeConstraint extends TypedConstraint {

    /**
     * 3 orthogonal linear constraints
     */
    private final JacobianEntry[] jac/*[3]*/ = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};

    /**
     * 2 orthogonal angular constraints+ 1 for limit/motor
     */
    private final JacobianEntry[] jacAng/*[3]*/ = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};

    /**
     * constraint axii. Assumes z is hinge axis.
     */
    private final Transform rbAFrame = new Transform();
    private final Transform rbBFrame = new Transform();

    private double motorTargetVelocity;
    private double maxMotorImpulse;

    private double limitSoftness;
    private double biasFactor;
    private double relaxationFactor;

    private double lowerLimit;
    private double upperLimit;

    private double kHinge;

    private double limitSign;
    private double correction;

    private double accLimitImpulse;

    private boolean angularOnly;
    private boolean enableAngularMotor;
    private boolean solveLimit;

    public HingeConstraint() {
        super(TypedConstraintType.HINGE_CONSTRAINT_TYPE);
        enableAngularMotor = false;
    }

    public HingeConstraint(RigidBody rbA, RigidBody rbB, Vector3d pivotInA, Vector3d pivotInB, Vector3d axisInA, Vector3d axisInB) {
        super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB);
        angularOnly = false;
        enableAngularMotor = false;

        rbAFrame.origin.set(pivotInA);

        // since no frame is given, assume this to be zero angle and just pick rb transform axis
        Vector3d rbAxisA1 = Stack.newVec();
        Vector3d rbAxisA2 = Stack.newVec();

        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        centerOfMassA.basis.getColumn(0, rbAxisA1);
        double projection = axisInA.dot(rbAxisA1);

        if (projection >= 1.0 - BulletGlobals.SIMD_EPSILON) {
            centerOfMassA.basis.getColumn(2, rbAxisA1);
            rbAxisA1.negate();
            centerOfMassA.basis.getColumn(1, rbAxisA2);
        } else if (projection <= -1.0 + BulletGlobals.SIMD_EPSILON) {
            centerOfMassA.basis.getColumn(2, rbAxisA1);
            centerOfMassA.basis.getColumn(1, rbAxisA2);
        } else {
            rbAxisA2.cross(axisInA, rbAxisA1);
            rbAxisA1.cross(rbAxisA2, axisInA);
        }

        rbAFrame.basis.setRow(0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
        rbAFrame.basis.setRow(1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
        rbAFrame.basis.setRow(2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

        Quat4d rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, Stack.newQuat());
        Vector3d rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, Stack.newVec());
        Vector3d rbAxisB2 = Stack.newVec();
        rbAxisB2.cross(axisInB, rbAxisB1);

        rbBFrame.origin.set(pivotInB);
        rbBFrame.basis.setRow(0, rbAxisB1.x, rbAxisB2.x, -axisInB.x);
        rbBFrame.basis.setRow(1, rbAxisB1.y, rbAxisB2.y, -axisInB.y);
        rbBFrame.basis.setRow(2, rbAxisB1.z, rbAxisB2.z, -axisInB.z);

        // start with free
        lowerLimit = 1e308;
        upperLimit = -1e308;
        biasFactor = 0.3;
        relaxationFactor = 1.0;
        limitSoftness = 0.9;
        solveLimit = false;
    }

    public HingeConstraint(RigidBody rbA, Vector3d pivotInA, Vector3d axisInA) {
        super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA);
        angularOnly = false;
        enableAngularMotor = false;

        // since no frame is given, assume this to be zero angle and just pick rb transform axis
        // fixed axis in worldspace
        Vector3d rbAxisA1 = Stack.newVec();
        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        centerOfMassA.basis.getColumn(0, rbAxisA1);

        double projection = rbAxisA1.dot(axisInA);
        if (projection > BulletGlobals.FLT_EPSILON) {
            rbAxisA1.scale(projection);
            rbAxisA1.sub(axisInA);
        } else {
            centerOfMassA.basis.getColumn(1, rbAxisA1);
        }

        Vector3d rbAxisA2 = Stack.newVec();
        rbAxisA2.cross(axisInA, rbAxisA1);

        rbAFrame.origin.set(pivotInA);
        rbAFrame.basis.setRow(0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
        rbAFrame.basis.setRow(1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
        rbAFrame.basis.setRow(2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

        Vector3d axisInB = Stack.newVec();
        axisInB.negate(axisInA);
        centerOfMassA.basis.transform(axisInB);

        Quat4d rotationArc = QuaternionUtil.shortestArcQuat(axisInA, axisInB, Stack.newQuat());
        Vector3d rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, Stack.newVec());
        Vector3d rbAxisB2 = Stack.newVec();
        rbAxisB2.cross(axisInB, rbAxisB1);

        rbBFrame.origin.set(pivotInA);
        centerOfMassA.transform(rbBFrame.origin);
        rbBFrame.basis.setRow(0, rbAxisB1.x, rbAxisB2.x, axisInB.x);
        rbBFrame.basis.setRow(1, rbAxisB1.y, rbAxisB2.y, axisInB.y);
        rbBFrame.basis.setRow(2, rbAxisB1.z, rbAxisB2.z, axisInB.z);

        // start with free
        lowerLimit = 1e308;
        upperLimit = -1e308;
        biasFactor = 0.3;
        relaxationFactor = 1.0;
        limitSoftness = 0.9;
        solveLimit = false;
    }

    public HingeConstraint(RigidBody rbA, RigidBody rbB, Transform rbAFrame, Transform rbBFrame) {
        super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA, rbB);
        this.rbAFrame.set(rbAFrame);
        this.rbBFrame.set(rbBFrame);
        angularOnly = false;
        enableAngularMotor = false;

        // flip axis
        this.rbBFrame.basis.m02 *= -1.0;
        this.rbBFrame.basis.m12 *= -1.0;
        this.rbBFrame.basis.m22 *= -1.0;

        // start with free
        lowerLimit = 1e308;
        upperLimit = -1e308;
        biasFactor = 0.3;
        relaxationFactor = 1.0;
        limitSoftness = 0.9;
        solveLimit = false;
    }

    public HingeConstraint(RigidBody rbA, Transform rbAFrame) {
        super(TypedConstraintType.HINGE_CONSTRAINT_TYPE, rbA);
        this.rbAFrame.set(rbAFrame);
        this.rbBFrame.set(rbAFrame);
        angularOnly = false;
        enableAngularMotor = false;

        // not providing rigidbody B means implicitly using worldspace for body B

        // flip axis
        this.rbBFrame.basis.m02 *= -1.0;
        this.rbBFrame.basis.m12 *= -1.0;
        this.rbBFrame.basis.m22 *= -1.0;

        this.rbBFrame.origin.set(this.rbAFrame.origin);
        rbA.getCenterOfMassTransform(Stack.newTrans()).transform(this.rbBFrame.origin);

        // start with free
        lowerLimit = 1e308;
        upperLimit = -1e308;
        biasFactor = 0.3;
        relaxationFactor = 1.0;
        limitSoftness = 0.9;
        solveLimit = false;
    }

    @Override
    public void buildJacobian() {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();
        Vector3d tmpVec = Stack.newVec();
        Matrix3d mat1 = Stack.newMat();
        Matrix3d mat2 = Stack.newMat();

        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

        appliedImpulse = 0.0;

        if (!angularOnly) {
            Vector3d pivotAInW = Stack.newVec(rbAFrame.origin);
            centerOfMassA.transform(pivotAInW);

            Vector3d pivotBInW = Stack.newVec(rbBFrame.origin);
            centerOfMassB.transform(pivotBInW);

            Vector3d relPos = Stack.newVec();
            relPos.sub(pivotBInW, pivotAInW);

            Vector3d[] normal/*[3]*/ = new Vector3d[]{Stack.newVec(), Stack.newVec(), Stack.newVec()};
            if (relPos.lengthSquared() > BulletGlobals.FLT_EPSILON) {
                normal[0].set(relPos);
                normal[0].normalize();
            } else {
                normal[0].set(1.0, 0.0, 0.0);
            }

            TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

            Vector3d tmp3 = Stack.newVec();
            Vector3d tmp4 = Stack.newVec();
            for (int i = 0; i < 3; i++) {
                mat1.transpose(centerOfMassA.basis);
                mat2.transpose(centerOfMassB.basis);

                tmp1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
                tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

                jac[i].init(
                        mat1,
                        mat2,
                        tmp1,
                        tmp2,
                        normal[i],
                        rbA.getInvInertiaDiagLocal(tmp3),
                        rbA.getInvMass(),
                        rbB.getInvInertiaDiagLocal(tmp4),
                        rbB.getInvMass());
            }
            Stack.subVec(6);
        }

        // calculate two perpendicular jointAxis, orthogonal to hingeAxis
        // these two jointAxis require equal angular velocities for both bodies

        // this is unused for now, it's a todo
        Vector3d jointAxis0local = Stack.newVec();
        Vector3d jointAxis1local = Stack.newVec();

        rbAFrame.basis.getColumn(2, tmp);
        TransformUtil.planeSpace1(tmp, jointAxis0local, jointAxis1local);

        // TODO: check this
        //getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);

        Vector3d jointAxis0 = Stack.newVec(jointAxis0local);
        centerOfMassA.basis.transform(jointAxis0);

        Vector3d jointAxis1 = Stack.newVec(jointAxis1local);
        centerOfMassA.basis.transform(jointAxis1);

        Vector3d hingeAxisWorld = Stack.newVec();
        rbAFrame.basis.getColumn(2, hingeAxisWorld);
        centerOfMassA.basis.transform(hingeAxisWorld);

        mat1.transpose(centerOfMassA.basis);
        mat2.transpose(centerOfMassB.basis);
        jacAng[0].init(jointAxis0,
                mat1,
                mat2,
                rbA.getInvInertiaDiagLocal(tmp1),
                rbB.getInvInertiaDiagLocal(tmp2));

        // JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
        jacAng[1].init(jointAxis1,
                mat1,
                mat2,
                rbA.getInvInertiaDiagLocal(tmp1),
                rbB.getInvInertiaDiagLocal(tmp2));

        // JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
        jacAng[2].init(hingeAxisWorld,
                mat1,
                mat2,
                rbA.getInvInertiaDiagLocal(tmp1),
                rbB.getInvInertiaDiagLocal(tmp2));

        // Compute limit information
        double hingeAngle = getHingeAngle();

        //set bias, sign, clear accumulator
        correction = 0.0;
        limitSign = 0.0;
        solveLimit = false;
        accLimitImpulse = 0.0;

        if (lowerLimit < upperLimit) {
            if (hingeAngle <= lowerLimit * limitSoftness) {
                correction = (lowerLimit - hingeAngle);
                limitSign = 1.0;
                solveLimit = true;
            } else if (hingeAngle >= upperLimit * limitSoftness) {
                correction = upperLimit - hingeAngle;
                limitSign = -1.0;
                solveLimit = true;
            }
        }

        // Compute K = J*W*J' for hinge axis
        Vector3d axisA = Stack.newVec();
        rbAFrame.basis.getColumn(2, axisA);
        centerOfMassA.basis.transform(axisA);

        kHinge = 1.0 / (getRigidBodyA().computeAngularImpulseDenominator(axisA) +
                getRigidBodyB().computeAngularImpulseDenominator(axisA));

        Stack.subVec(10);
        Stack.subMat(2);
        Stack.subTrans(2);
    }

    @Override
    public void solveConstraint(double timeStep) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();
        Vector3d tmpVec = Stack.newVec();

        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

        Vector3d pivotAInW = Stack.newVec(rbAFrame.origin);
        centerOfMassA.transform(pivotAInW);

        Vector3d pivotBInW = Stack.newVec(rbBFrame.origin);
        centerOfMassB.transform(pivotBInW);

        double tau = 0.3;

        // linear part
        if (!angularOnly) {
            Vector3d relPos1 = Stack.newVec();
            relPos1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));

            Vector3d relPos2 = Stack.newVec();
            relPos2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

            Vector3d vel1 = rbA.getVelocityInLocalPoint(relPos1, Stack.newVec());
            Vector3d vel2 = rbB.getVelocityInLocalPoint(relPos2, Stack.newVec());
            Vector3d vel = Stack.newVec();
            vel.sub(vel1, vel2);

            Vector3d impulseVector = Stack.newVec();
            for (int i = 0; i < 3; i++) {
                Vector3d normal = jac[i].linearJointAxis;
                double jacDiagABInv = 1.0 / jac[i].getDiagonal();

                double rel_vel;
                rel_vel = normal.dot(vel);
                // positional error (zeroth order error)
                tmp.sub(pivotAInW, pivotBInW);
                double depth = -(tmp).dot(normal); // this is the error projected on the normal
                double impulse = depth * tau / timeStep * jacDiagABInv - rel_vel * jacDiagABInv;
                if (impulse > getBreakingImpulseThreshold()) {
                    setBroken(true);
                    break;
                }

                appliedImpulse += impulse;
                impulseVector.scale(impulse, normal);

                tmp.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
                rbA.applyImpulse(impulseVector, tmp);

                tmp.negate(impulseVector);
                tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
                rbB.applyImpulse(tmp, tmp2);
            }
            Stack.subVec(6);
        }

        {
            // solve angular part

            // get axes in world space
            Vector3d axisA = Stack.newVec();
            rbAFrame.basis.getColumn(2, axisA);
            centerOfMassA.basis.transform(axisA);

            Vector3d axisB = Stack.newVec();
            rbBFrame.basis.getColumn(2, axisB);
            centerOfMassB.basis.transform(axisB);

            Vector3d angVelA = getRigidBodyA().getAngularVelocity(Stack.newVec());
            Vector3d angVelB = getRigidBodyB().getAngularVelocity(Stack.newVec());

            Vector3d angVelAroundHingeAxisA = Stack.newVec();
            angVelAroundHingeAxisA.scale(axisA.dot(angVelA), axisA);

            Vector3d angVelAroundHingeAxisB = Stack.newVec();
            angVelAroundHingeAxisB.scale(axisB.dot(angVelB), axisB);

            Vector3d angleAOrthogonal = Stack.newVec();
            angleAOrthogonal.sub(angVelA, angVelAroundHingeAxisA);

            Vector3d angleBOrthogonal = Stack.newVec();
            angleBOrthogonal.sub(angVelB, angVelAroundHingeAxisB);

            Vector3d velRelOrthogonal = Stack.newVec();
            velRelOrthogonal.sub(angleAOrthogonal, angleBOrthogonal);

            {
                // solve orthogonal angular velocity correction
                double relaxation = 1.0;
                double len = velRelOrthogonal.length();
                if (len > 0.00001f) {
                    Vector3d normal = Stack.newVec();
                    normal.normalize(velRelOrthogonal);

                    double denominator = getRigidBodyA().computeAngularImpulseDenominator(normal) +
                            getRigidBodyB().computeAngularImpulseDenominator(normal);
                    // scale for mass and relaxation
                    velRelOrthogonal.scale((1.0 / denominator) * relaxationFactor);
                    Stack.subVec(1);
                }

                // solve angular positional correction
                // TODO: check
                //Vector3d angularError = -axisA.cross(axisB) *(btScalar(1.)/timeStep);
                Vector3d angularError = Stack.newVec();
                angularError.cross(axisA, axisB);
                angularError.negate();
                angularError.scale(1.0 / timeStep);
                double len2 = angularError.length();
                if (len2 > 0.00001) {
                    Vector3d normal2 = Stack.newVec();
                    normal2.normalize(angularError);

                    double denominator = getRigidBodyA().computeAngularImpulseDenominator(normal2) +
                            getRigidBodyB().computeAngularImpulseDenominator(normal2);
                    angularError.scale((1.0 / denominator) * relaxation);
                }

                tmp.negate(velRelOrthogonal);
                tmp.add(angularError);
                rbA.applyTorqueImpulse(tmp);

                tmp.sub(velRelOrthogonal, angularError);
                rbB.applyTorqueImpulse(tmp);

                // solve limit
                if (solveLimit) {
                    tmp.sub(angVelB, angVelA);
                    double amplitude = ((tmp).dot(axisA) * relaxationFactor + correction * (1.0 / timeStep) * biasFactor) * limitSign;

                    double impulseMag = amplitude * kHinge;
                    if (Math.abs(impulseMag) > getBreakingImpulseThreshold()) {
                        setBroken(true);
                    } else {
                        // Clamp the accumulated impulse
                        double temp = accLimitImpulse;
                        accLimitImpulse = Math.max(accLimitImpulse + impulseMag, 0.0);
                        impulseMag = accLimitImpulse - temp;

                        Vector3d impulse = Stack.newVec();
                        impulse.scale(impulseMag * limitSign, axisA);

                        rbA.applyTorqueImpulse(impulse);

                        tmp.negate(impulse);
                        rbB.applyTorqueImpulse(tmp);
                        Stack.subVec(1); // impulse
                    }
                }
            }

            // apply motor
            if (enableAngularMotor) {
                // todo: add limits too
                Vector3d angularLimit = Stack.newVec();
                angularLimit.set(0.0, 0.0, 0.0);

                Vector3d velRel = Stack.newVec();
                velRel.sub(angVelAroundHingeAxisA, angVelAroundHingeAxisB);
                double projRelVel = velRel.dot(axisA);

                double desiredMotorVel = motorTargetVelocity;
                double motorRelVel = desiredMotorVel - projRelVel;

                double unclippedMotorImpulse = kHinge * motorRelVel;
                if (unclippedMotorImpulse > getBreakingImpulseThreshold()) {
                    setBroken(true);
                    Stack.subVec(2); // angularLimit, velrel
                } else {
                    // todo: should clip against accumulated impulse
                    double clippedMotorImpulse = Math.min(unclippedMotorImpulse, maxMotorImpulse);
                    clippedMotorImpulse = Math.max(clippedMotorImpulse, -maxMotorImpulse);
                    Vector3d motorImp = Stack.newVec();
                    motorImp.scale(clippedMotorImpulse, axisA);

                    tmp.add(motorImp, angularLimit);
                    rbA.applyTorqueImpulse(tmp);

                    tmp.negate(motorImp);
                    tmp.sub(angularLimit);
                    rbB.applyTorqueImpulse(tmp);
                    Stack.subVec(3); // angularLimit, velrel, motorImp
                }
            }

            Stack.subVec(9);
        }

        Stack.subVec(5);
        Stack.subTrans(2);
    }

    public void updateRHS(double timeStep) {
    }

    public double getHingeAngle() {
        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

        Vector3d refAxis0 = Stack.newVec();
        rbAFrame.basis.getColumn(0, refAxis0);
        centerOfMassA.basis.transform(refAxis0);

        Vector3d refAxis1 = Stack.newVec();
        rbAFrame.basis.getColumn(1, refAxis1);
        centerOfMassA.basis.transform(refAxis1);

        Vector3d swingAxis = Stack.newVec();
        rbBFrame.basis.getColumn(1, swingAxis);
        centerOfMassB.basis.transform(swingAxis);

        double hingeAngle = ScalarUtil.atan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
        Stack.subVec(3);
        Stack.subTrans(2);
        return hingeAngle;
    }

    @SuppressWarnings("unused")
    public void setAngularOnly(boolean angularOnly) {
        this.angularOnly = angularOnly;
    }

    public void enableAngularMotor(boolean enableMotor, double targetVelocity, double maxMotorImpulse) {
        this.enableAngularMotor = enableMotor;
        this.motorTargetVelocity = targetVelocity;
        this.maxMotorImpulse = maxMotorImpulse;
    }

    public void setLimit(double low, double high) {
        setLimit(low, high, 0.9, 0.3, 1.0);
    }

    public void setLimit(double low, double high, double _softness, double _biasFactor, double _relaxationFactor) {
        lowerLimit = low;
        upperLimit = high;

        limitSoftness = _softness;
        biasFactor = _biasFactor;
        relaxationFactor = _relaxationFactor;
    }

    @SuppressWarnings("unused")
    public double getLowerLimit() {
        return lowerLimit;
    }

    @SuppressWarnings("unused")
    public double getUpperLimit() {
        return upperLimit;
    }

    @SuppressWarnings("unused")
    public Transform getAFrame(Transform out) {
        out.set(rbAFrame);
        return out;
    }

    @SuppressWarnings("unused")
    public Transform getBFrame(Transform out) {
        out.set(rbBFrame);
        return out;
    }

    @SuppressWarnings("unused")
    public boolean getSolveLimit() {
        return solveLimit;
    }

    @SuppressWarnings("unused")
    public double getLimitSign() {
        return limitSign;
    }

    @SuppressWarnings("unused")
    public boolean getAngularOnly() {
        return angularOnly;
    }

    @SuppressWarnings("unused")
    public boolean getEnableAngularMotor() {
        return enableAngularMotor;
    }

    @SuppressWarnings("unused")
    public double getMotorTargetVelocity() {
        return motorTargetVelocity;
    }

    @SuppressWarnings("unused")
    public double getMaxMotorImpulse() {
        return maxMotorImpulse;
    }

}
