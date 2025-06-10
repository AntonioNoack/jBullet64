package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Point to point constraint between two rigid bodies each with a pivot point that
 * describes the "ballsocket" location in local space.
 *
 * @author jezek2
 */
public class Point2PointConstraint extends TypedConstraint {

    private final JacobianEntry[] jac = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; // 3 orthogonal linear constraints

    private final Vector3d pivotInA = new Vector3d();
    private final Vector3d pivotInB = new Vector3d();

    public ConstraintSetting setting = new ConstraintSetting();

    @SuppressWarnings("unused")
    public Point2PointConstraint() {
        super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE);
    }

    public Point2PointConstraint(RigidBody rbA, RigidBody rbB, Vector3d pivotInA, Vector3d pivotInB) {
        super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE, rbA, rbB);
        this.pivotInA.set(pivotInA);
        this.pivotInB.set(pivotInB);
    }

    @SuppressWarnings("unused")
    public Point2PointConstraint(RigidBody rbA, Vector3d pivotInA) {
        super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE, rbA);
        this.pivotInA.set(pivotInA);
        this.pivotInB.set(pivotInA);
        rbA.getCenterOfMassTransform(Stack.newTrans()).transform(this.pivotInB);
    }

    @Override
    public void buildJacobian() {
        appliedImpulse = 0.0;

        Vector3d normal = Stack.newVec();
        normal.set(0.0, 0.0, 0.0);

        Matrix3d tmpMat1 = Stack.newMat();
        Matrix3d tmpMat2 = Stack.newMat();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();
        Vector3d tmpVec = Stack.newVec();

        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

        for (int i = 0; i < 3; i++) {
            VectorUtil.setCoord(normal, i, 1.0);

            tmpMat1.transpose(centerOfMassA.basis);
            tmpMat2.transpose(centerOfMassB.basis);

            tmp1.set(pivotInA);
            centerOfMassA.transform(tmp1);
            tmp1.sub(rbA.getCenterOfMassPosition(tmpVec));

            tmp2.set(pivotInB);
            centerOfMassB.transform(tmp2);
            tmp2.sub(rbB.getCenterOfMassPosition(tmpVec));

            jac[i].init(
                    tmpMat1, tmpMat2, tmp1, tmp2, normal,
                    rbA.getInvInertiaDiagLocal(Stack.newVec()),
                    rbA.getInvMass(),
                    rbB.getInvInertiaDiagLocal(Stack.newVec()),
                    rbB.getInvMass());
            VectorUtil.setCoord(normal, i, 0.0);
        }
    }

    @Override
    public void solveConstraint(double timeStep) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();
        Vector3d tmpVec = Stack.newVec();

        Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
        Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

        Vector3d pivotAInW = Stack.newVec(pivotInA);
        centerOfMassA.transform(pivotAInW);

        Vector3d pivotBInW = Stack.newVec(pivotInB);
        centerOfMassB.transform(pivotBInW);

        Vector3d normal = Stack.newVec();
        normal.set(0.0, 0.0, 0.0);

        //btVector3 angvelA = m_rbA.getCenterOfMassTransform().getBasis().transpose() * m_rbA.getAngularVelocity();
        //btVector3 angvelB = m_rbB.getCenterOfMassTransform().getBasis().transpose() * m_rbB.getAngularVelocity();

        Vector3d relPos1 = Stack.newVec();
        Vector3d relPos2 = Stack.newVec();
        Vector3d vel1 = Stack.newVec();
        Vector3d vel2 = Stack.newVec();
        Vector3d vel = Stack.newVec();
        Vector3d impulseVector = Stack.newVec();

        for (int i = 0; i < 3; i++) {
            VectorUtil.setCoord(normal, i, 1.0);
            double jacDiagABInv = 1.0 / jac[i].getDiagonal();

            relPos1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
            relPos2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
            // this jacobian entry could be re-used for all iterations

            rbA.getVelocityInLocalPoint(relPos1, vel1);
            rbB.getVelocityInLocalPoint(relPos2, vel2);
            vel.sub(vel1, vel2);

            double relativeVelocity;
            relativeVelocity = normal.dot(vel);

			/*
			//velocity error (first order error)
			btScalar rel_vel = m_jac[i].getRelativeVelocity(m_rbA.getLinearVelocity(),angvelA,
			m_rbB.getLinearVelocity(),angvelB);
			 */

            // positional error (zeroth order error)
            tmp.sub(pivotAInW, pivotBInW);
            double depth = -tmp.dot(normal); //this is the error projected on the normal

            double impulse = depth * setting.tau / timeStep * jacDiagABInv - setting.damping * relativeVelocity * jacDiagABInv;
            if (Math.abs(impulse) > getBreakingImpulseThreshold()) {
                setBroken(true);
                break;
            }

            double impulseClamp = setting.impulseClamp;
            if (impulseClamp > 0.0) {
                if (impulse < -impulseClamp) {
                    impulse = -impulseClamp;
                }
                if (impulse > impulseClamp) {
                    impulse = impulseClamp;
                }
            }

            appliedImpulse += impulse;
            impulseVector.scale(impulse, normal);
            tmp.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
            rbA.applyImpulse(impulseVector, tmp);
            tmp.negate(impulseVector);
            tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
            rbB.applyImpulse(tmp, tmp2);

            VectorUtil.setCoord(normal, i, 0.0);
        }

        Stack.subVec(12);
        Stack.subTrans(2);
    }

    public void updateRHS(double timeStep) {
    }

    public void setPivotA(Vector3d pivotA) {
        pivotInA.set(pivotA);
    }

    public void setPivotB(Vector3d pivotB) {
        pivotInB.set(pivotB);
    }

    public Vector3d getPivotInA(Vector3d out) {
        out.set(pivotInA);
        return out;
    }

    public Vector3d getPivotInB(Vector3d out) {
        out.set(pivotInB);
        return out;
    }

    /// /////////////////////////////////////////////////////////////////////////

    public static class ConstraintSetting {
        public double tau = 0.3;
        public double damping = 1.0;
        public double impulseClamp = 0.0;
    }
}
