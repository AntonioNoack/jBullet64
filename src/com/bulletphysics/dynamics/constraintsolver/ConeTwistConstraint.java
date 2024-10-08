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
 * ConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
 *
 * @author jezek2
 */
public class ConeTwistConstraint extends TypedConstraint {

    private final JacobianEntry[] jac = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; //3 orthogonal linear constraints

    private final Transform rbAFrame = new Transform();
    private final Transform rbBFrame = new Transform();

    private double limitSoftness;
    private double biasFactor;
    private double relaxationFactor;

    private double swingSpan1;
    private double swingSpan2;
    private double twistSpan;

    private final Vector3d swingAxis = new Vector3d();
    private final Vector3d twistAxis = new Vector3d();

    private double kSwing;
    private double kTwist;

    private double twistLimitSign;
    private double swingCorrection;
    private double twistCorrection;

    private double accSwingLimitImpulse;
    private double accTwistLimitImpulse;

    private boolean angularOnly = false;
    private boolean solveTwistLimit;
    private boolean solveSwingLimit;

    public ConeTwistConstraint() {
        super(TypedConstraintType.CONE_TWIST_CONSTRAINT_TYPE);
    }

    public ConeTwistConstraint(RigidBody rbA, RigidBody rbB, Transform rbAFrame, Transform rbBFrame) {
        super(TypedConstraintType.CONE_TWIST_CONSTRAINT_TYPE, rbA, rbB);
        this.rbAFrame.set(rbAFrame);
        this.rbBFrame.set(rbBFrame);

        swingSpan1 = 1e30;
        swingSpan2 = 1e30;
        twistSpan = 1e30;
        biasFactor = 0.3;
        relaxationFactor = 1.0;

        solveTwistLimit = false;
        solveSwingLimit = false;
    }

    public ConeTwistConstraint(RigidBody rbA, Transform rbAFrame) {
        super(TypedConstraintType.CONE_TWIST_CONSTRAINT_TYPE, rbA);
        this.rbAFrame.set(rbAFrame);
        this.rbBFrame.set(this.rbAFrame);

        swingSpan1 = 1e30;
        swingSpan2 = 1e30;
        twistSpan = 1e30;
        biasFactor = 0.3;
        relaxationFactor = 1.0;

        solveTwistLimit = false;
        solveSwingLimit = false;
    }

    @Override
    public void buildJacobian() {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        Transform tmpTrans = Stack.newTrans();

        appliedImpulse = 0.0;

        // set bias, sign, clear accumulator
        swingCorrection = 0.0;
        twistLimitSign = 0.0;
        solveTwistLimit = false;
        solveSwingLimit = false;
        accTwistLimitImpulse = 0.0;
        accSwingLimitImpulse = 0.0;

        if (!angularOnly) {
            Vector3d pivotAInW = Stack.newVec(rbAFrame.origin);
            rbA.getCenterOfMassTransform(tmpTrans).transform(pivotAInW);

            Vector3d pivotBInW = Stack.newVec(rbBFrame.origin);
            rbB.getCenterOfMassTransform(tmpTrans).transform(pivotBInW);

            Vector3d relPos = Stack.newVec();
            relPos.sub(pivotBInW, pivotAInW);

            // TODO: stack
            Vector3d[] normal/*[3]*/ = new Vector3d[]{Stack.newVec(), Stack.newVec(), Stack.newVec()};
            if (relPos.lengthSquared() > BulletGlobals.FLT_EPSILON) {
                normal[0].normalize(relPos);
            } else {
                normal[0].set(1.0, 0.0, 0.0);
            }

            TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

            for (int i = 0; i < 3; i++) {
                Matrix3d mat1 = rbA.getCenterOfMassTransform(Stack.newTrans()).basis;
                mat1.transpose();

                Matrix3d mat2 = rbB.getCenterOfMassTransform(Stack.newTrans()).basis;
                mat2.transpose();

                tmp1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmp));
                tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmp));

                jac[i].init(
                        mat1,
                        mat2,
                        tmp1,
                        tmp2,
                        normal[i],
                        rbA.getInvInertiaDiagLocal(Stack.newVec()),
                        rbA.getInvMass(),
                        rbB.getInvInertiaDiagLocal(Stack.newVec()),
                        rbB.getInvMass());
            }
        }

        Vector3d b1Axis1 = Stack.newVec(), b1Axis2 = Stack.newVec(), b1Axis3 = Stack.newVec();
        Vector3d b2Axis1 = Stack.newVec(), b2Axis2 = Stack.newVec();

        rbAFrame.basis.getColumn(0, b1Axis1);
        getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis.transform(b1Axis1);

        rbBFrame.basis.getColumn(0, b2Axis1);
        getRigidBodyB().getCenterOfMassTransform(tmpTrans).basis.transform(b2Axis1);

        double swing1 = 0.0, swing2 = 0.0;

        double swx, swy;
        double thresh = 10f;
        double fact;

        // Get Frame into world space
        if (swingSpan1 >= 0.05f) {
            rbAFrame.basis.getColumn(1, b1Axis2);
            getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis.transform(b1Axis2);
//			swing1 = ScalarUtil.atan2Fast(b2Axis1.dot(b1Axis2), b2Axis1.dot(b1Axis1));
            swx = b2Axis1.dot(b1Axis1);
            swy = b2Axis1.dot(b1Axis2);
            swing1 = ScalarUtil.atan2Fast(swy, swx);
            fact = (swy * swy + swx * swx) * thresh * thresh;
            fact = fact / (fact + 1.0);
            swing1 *= fact;
        }

        if (swingSpan2 >= 0.05f) {
            rbAFrame.basis.getColumn(2, b1Axis3);
            getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis.transform(b1Axis3);
            swx = b2Axis1.dot(b1Axis1);
            swy = b2Axis1.dot(b1Axis3);
            swing2 = ScalarUtil.atan2Fast(swy, swx);
            fact = (swy * swy + swx * swx) * thresh * thresh;
            fact = fact / (fact + 1.0);
            swing2 *= fact;
        }

        double RMaxAngle1Sq = 1.0 / (swingSpan1 * swingSpan1);
        double RMaxAngle2Sq = 1.0 / (swingSpan2 * swingSpan2);
        double EllipseAngle = Math.abs(swing1 * swing1) * RMaxAngle1Sq + Math.abs(swing2 * swing2) * RMaxAngle2Sq;

        if (EllipseAngle > 1.0) {
            swingCorrection = EllipseAngle - 1.0;
            solveSwingLimit = true;

            // Calculate necessary axis & factors
            tmp1.scale(b2Axis1.dot(b1Axis2), b1Axis2);
            tmp2.scale(b2Axis1.dot(b1Axis3), b1Axis3);
            tmp.add(tmp1, tmp2);
            swingAxis.cross(b2Axis1, tmp);
            swingAxis.normalize();

            double swingAxisSign = (b2Axis1.dot(b1Axis1) >= 0.0) ? 1.0 : -1.0;
            swingAxis.scale(swingAxisSign);

            kSwing = 1.0 / (getRigidBodyA().computeAngularImpulseDenominator(swingAxis) +
                    getRigidBodyB().computeAngularImpulseDenominator(swingAxis));

        }

        // Twist limits
        if (twistSpan >= 0.0) {
            rbBFrame.basis.getColumn(1, b2Axis2);
            getRigidBodyB().getCenterOfMassTransform(tmpTrans).basis.transform(b2Axis2);

            Quat4d rotationArc = QuaternionUtil.shortestArcQuat(b2Axis1, b1Axis1, Stack.newQuat());
            Vector3d TwistRef = QuaternionUtil.quatRotate(rotationArc, b2Axis2, Stack.newVec());
            double twist = ScalarUtil.atan2Fast(TwistRef.dot(b1Axis3), TwistRef.dot(b1Axis2));

            double lockedFreeFactor = (twistSpan > 0.05f) ? limitSoftness : 0.0;
            if (twist <= -twistSpan * lockedFreeFactor) {
                twistCorrection = -(twist + twistSpan);
                solveTwistLimit = true;

                twistAxis.add(b2Axis1, b1Axis1);
                twistAxis.scale(0.5);
                twistAxis.normalize();
                twistAxis.scale(-1.0);

                kTwist = 1.0 / (getRigidBodyA().computeAngularImpulseDenominator(twistAxis) +
                        getRigidBodyB().computeAngularImpulseDenominator(twistAxis));

            } else if (twist > twistSpan * lockedFreeFactor) {
                twistCorrection = (twist - twistSpan);
                solveTwistLimit = true;

                twistAxis.add(b2Axis1, b1Axis1);
                twistAxis.scale(0.5);
                twistAxis.normalize();

                kTwist = 1.0 / (getRigidBodyA().computeAngularImpulseDenominator(twistAxis) +
                        getRigidBodyB().computeAngularImpulseDenominator(twistAxis));
            }
        }
    }

    @Override
    public void solveConstraint(double timeStep) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        Vector3d tmpVec = Stack.newVec();
        Transform tmpTrans = Stack.newTrans();

        Vector3d pivotAInW = Stack.newVec(rbAFrame.origin);
        rbA.getCenterOfMassTransform(tmpTrans).transform(pivotAInW);

        Vector3d pivotBInW = Stack.newVec(rbBFrame.origin);
        rbB.getCenterOfMassTransform(tmpTrans).transform(pivotBInW);

        double tau = 0.3;

        // linear part
        if (!angularOnly) {
            Vector3d rel_pos1 = Stack.newVec();
            rel_pos1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));

            Vector3d rel_pos2 = Stack.newVec();
            rel_pos2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

            Vector3d vel1 = rbA.getVelocityInLocalPoint(rel_pos1, Stack.newVec());
            Vector3d vel2 = rbB.getVelocityInLocalPoint(rel_pos2, Stack.newVec());
            Vector3d vel = Stack.newVec();
            vel.sub(vel1, vel2);

            for (int i = 0; i < 3; i++) {
                Vector3d normal = jac[i].linearJointAxis;
                double jacDiagABInv = 1.0 / jac[i].getDiagonal();

                double rel_vel;
                rel_vel = normal.dot(vel);
                // positional error (zeroth order error)
                tmp.sub(pivotAInW, pivotBInW);
                double depth = -(tmp).dot(normal); // this is the error projected on the normal
                double impulse = depth * tau / timeStep * jacDiagABInv - rel_vel * jacDiagABInv;
                appliedImpulse += impulse;
                Vector3d impulse_vector = Stack.newVec();
                impulse_vector.scale(impulse, normal);

                tmp.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
                rbA.applyImpulse(impulse_vector, tmp);

                tmp.negate(impulse_vector);
                tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
                rbB.applyImpulse(tmp, tmp2);
            }
        }

        {
            // solve angular part
            Vector3d angVelA = getRigidBodyA().getAngularVelocity(Stack.newVec());
            Vector3d angVelB = getRigidBodyB().getAngularVelocity(Stack.newVec());

            // solve swing limit
            if (solveSwingLimit) {
                tmp.sub(angVelB, angVelA);
                double amplitude = ((tmp).dot(swingAxis) * relaxationFactor * relaxationFactor + swingCorrection * (1.0 / timeStep) * biasFactor);
                double impulseMag = amplitude * kSwing;

                // Clamp the accumulated impulse
                double temp = accSwingLimitImpulse;
                accSwingLimitImpulse = Math.max(accSwingLimitImpulse + impulseMag, 0.0);
                impulseMag = accSwingLimitImpulse - temp;

                Vector3d impulse = Stack.newVec();
                impulse.scale(impulseMag, swingAxis);

                rbA.applyTorqueImpulse(impulse);

                tmp.negate(impulse);
                rbB.applyTorqueImpulse(tmp);
            }

            // solve twist limit
            if (solveTwistLimit) {
                tmp.sub(angVelB, angVelA);
                double amplitude = ((tmp).dot(twistAxis) * relaxationFactor * relaxationFactor + twistCorrection * (1.0 / timeStep) * biasFactor);
                double impulseMag = amplitude * kTwist;

                // Clamp the accumulated impulse
                double temp = accTwistLimitImpulse;
                accTwistLimitImpulse = Math.max(accTwistLimitImpulse + impulseMag, 0.0);
                impulseMag = accTwistLimitImpulse - temp;

                Vector3d impulse = Stack.newVec();
                impulse.scale(impulseMag, twistAxis);

                rbA.applyTorqueImpulse(impulse);

                tmp.negate(impulse);
                rbB.applyTorqueImpulse(tmp);
            }
        }
    }

    public void updateRHS(double timeStep) {
    }

    public void setAngularOnly(boolean angularOnly) {
        this.angularOnly = angularOnly;
    }

    public void setLimit(double _swingSpan1, double _swingSpan2, double _twistSpan) {
        setLimit(_swingSpan1, _swingSpan2, _twistSpan, 0.8, 0.3, 1.0);
    }

    public void setLimit(double _swingSpan1, double _swingSpan2, double _twistSpan, double _softness, double _biasFactor, double _relaxationFactor) {
        swingSpan1 = _swingSpan1;
        swingSpan2 = _swingSpan2;
        twistSpan = _twistSpan;

        limitSoftness = _softness;
        biasFactor = _biasFactor;
        relaxationFactor = _relaxationFactor;
    }

    public Transform getAFrame(Transform out) {
        out.set(rbAFrame);
        return out;
    }

    public Transform getBFrame(Transform out) {
        out.set(rbBFrame);
        return out;
    }

    public boolean getSolveTwistLimit() {
        return solveTwistLimit;
    }

    public boolean getSolveSwingLimit() {
        return solveTwistLimit;
    }

    public double getTwistLimitSign() {
        return twistLimitSign;
    }

}
