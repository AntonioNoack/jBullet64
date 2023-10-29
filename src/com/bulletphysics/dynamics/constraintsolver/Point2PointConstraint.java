package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Point to point constraint between two rigid bodies each with a pivot point that
 * descibes the "ballsocket" location in local space.
 *
 * @author jezek2
 */
@SuppressWarnings("unused")
public class Point2PointConstraint extends TypedConstraint {

	private final JacobianEntry[] jac = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; // 3 orthogonal linear constraints

	private final Vector3d pivotInA = new Vector3d();
	private final Vector3d pivotInB = new Vector3d();

	public ConstraintSetting setting = new ConstraintSetting();

	public Point2PointConstraint() {
		super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE);
	}

	public Point2PointConstraint(RigidBody rbA, RigidBody rbB, Vector3d pivotInA, Vector3d pivotInB) {
		super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE, rbA, rbB);
		this.pivotInA.set(pivotInA);
		this.pivotInB.set(pivotInB);
	}

	public Point2PointConstraint(RigidBody rbA, Vector3d pivotInA) {
		super(TypedConstraintType.POINT2POINT_CONSTRAINT_TYPE, rbA);
		this.pivotInA.set(pivotInA);
		this.pivotInB.set(pivotInA);
		rbA.getCenterOfMassTransform(new Transform()).transform(this.pivotInB);
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
		Vector3d tmp3 = Stack.newVec();
		Vector3d tmp4 = Stack.newVec();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

		for (int i = 0; i < 3; i++) {
			VectorUtil.setCoord(normal, i, 1.0);

			tmpMat1.transpose(centerOfMassA.basis);
			tmpMat2.transpose(centerOfMassB.basis);

			tmp1.set(pivotInA);
			centerOfMassA.transform(tmp1);
			tmp1.sub(rbA.getCenterOfMassPosition(tmp3));

			tmp2.set(pivotInB);
			centerOfMassB.transform(tmp2);
			tmp2.sub(rbB.getCenterOfMassPosition(tmp3));

			jac[i].init(
					tmpMat1, tmpMat2,
					tmp1, tmp2, normal,
					rbA.getInvInertiaDiagLocal(tmp3),
					rbA.getInvMass(),
					rbB.getInvInertiaDiagLocal(tmp4),
					rbB.getInvMass());
			VectorUtil.setCoord(normal, i, 0.0);
		}

		Stack.subTrans(2);
		Stack.subMat(2);
		Stack.subVec(5);

	}

	@Override
	public void solveConstraint(double timeStep) {

		Transform centerOfMassA = rbA.getCenterOfMassTransform(Stack.newTrans());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Stack.newTrans());

		Vector3d pivotAInW = Stack.newVec(pivotInA);
		Vector3d pivotBInW = Stack.newVec(pivotInB);

		centerOfMassA.transform(pivotAInW);
		centerOfMassB.transform(pivotBInW);

		Vector3d normal = Stack.newVec();
		normal.set(0.0, 0.0, 0.0);

		Vector3d rel_pos1 = Stack.newVec();
		Vector3d rel_pos2 = Stack.newVec();

		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();
		Vector3d tmp3 = Stack.newVec();

		Vector3d vel1 = Stack.newVec();
		Vector3d vel2 = Stack.newVec();

		Vector3d impulse_vector = Stack.newVec();

		for (int i = 0; i < 3; i++) {
			VectorUtil.setCoord(normal, i, 1.0);
			double jacDiagABInv = 1.0 / jac[i].getDiagonal();

			rel_pos1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmp3));
			rel_pos2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmp3));

			// this jacobian entry could be re-used for all iterations
			rbA.getVelocityInLocalPoint(rel_pos1, vel1);
			rbB.getVelocityInLocalPoint(rel_pos2, vel2);
			vel1.sub(vel2);

			double rel_vel = normal.dot(vel1);

			// positional error (zeroth order error)
			tmp1.sub(pivotAInW, pivotBInW);
			double depth = -tmp1.dot(normal); //this is the error projected on the normal

			double impulse = depth * setting.tau / timeStep * jacDiagABInv - setting.damping * rel_vel * jacDiagABInv;
			double impulseClamp = setting.impulseClamp;
			if (impulseClamp > 0.0) {
				impulse = Math.max(impulse, -impulseClamp);
				impulse = Math.min(impulse, +impulseClamp);
			}

			appliedImpulse += impulse;
			impulse_vector.scale(impulse, normal);
			tmp1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmp3));
			rbA.applyImpulse(impulse_vector, tmp1);
			tmp1.negate(impulse_vector);
			tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmp3));
			rbB.applyImpulse(tmp1, tmp2);

			VectorUtil.setCoord(normal, i, 0.0);
		}

		Stack.subTrans(2);
		Stack.subVec(11);
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

	public static class ConstraintSetting {
		public double tau = 0.3;
		public double damping = 1.0;
		public double impulseClamp = 0.0;
	}

}
