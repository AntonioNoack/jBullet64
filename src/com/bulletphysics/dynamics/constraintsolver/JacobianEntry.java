package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/**
 * Jacobian entry is an abstraction that allows to describe constraints.
 * It can be used in combination with a constraint solver.
 * Can be used to relate the effect of an impulse to the constraint error.
 * 
 * @author jezek2
 */
public class JacobianEntry {

	public final Vector3d linearJointAxis = new Vector3d();
	public final Vector3d aJ = new Vector3d();
	public final Vector3d bJ = new Vector3d();
	public final Vector3d m_0MinvJt = new Vector3d();
	public final Vector3d m_1MinvJt = new Vector3d();
	// Optimization: can be stored in the w/last component of one of the vectors
	public double Adiag;

	public JacobianEntry() {
	}

	/**
	 * Constraint between two different rigidbodies.
	 */
	public void init(Matrix3d world2A,
			Matrix3d world2B,
			Vector3d rel_pos1, Vector3d rel_pos2,
			Vector3d jointAxis,
			Vector3d inertiaInvA,
			double massInvA,
			Vector3d inertiaInvB,
			double massInvB)
	{
		linearJointAxis.set(jointAxis);

		aJ.cross(rel_pos1, linearJointAxis);
		world2A.transform(aJ);

		bJ.set(linearJointAxis);
		bJ.negate();
		bJ.cross(rel_pos2, bJ);
		world2B.transform(bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = massInvA + m_0MinvJt.dot(aJ) + massInvB + m_1MinvJt.dot(bJ);

		assert (Adiag > 0.0);
	}

	/**
	 * Angular constraint between two different rigidbodies.
	 */
	public void init(Vector3d jointAxis,
		Matrix3d world2A,
		Matrix3d world2B,
		Vector3d inertiaInvA,
		Vector3d inertiaInvB)
	{
		linearJointAxis.set(0.0, 0.0, 0.0);

		aJ.set(jointAxis);
		world2A.transform(aJ);

		bJ.set(jointAxis);
		bJ.negate();
		world2B.transform(bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0.0);
	}

	/**
	 * Angular constraint between two different rigidbodies.
	 */
	public void init(Vector3d axisInA,
		Vector3d axisInB,
		Vector3d inertiaInvA,
		Vector3d inertiaInvB)
	{
		linearJointAxis.set(0.0, 0.0, 0.0);
		aJ.set(axisInA);

		bJ.set(axisInB);
		bJ.negate();

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		VectorUtil.mul(m_1MinvJt, inertiaInvB, bJ);
		Adiag = m_0MinvJt.dot(aJ) + m_1MinvJt.dot(bJ);

		assert (Adiag > 0.0);
	}

	/**
	 * Constraint on one rigidbody.
	 */
	public void init(
		Matrix3d world2A,
		Vector3d rel_pos1, Vector3d rel_pos2,
		Vector3d jointAxis,
		Vector3d inertiaInvA,
		double massInvA)
	{
		linearJointAxis.set(jointAxis);

		aJ.cross(rel_pos1, jointAxis);
		world2A.transform(aJ);

		bJ.set(jointAxis);
		bJ.negate();
		bJ.cross(rel_pos2, bJ);
		world2A.transform(bJ);

		VectorUtil.mul(m_0MinvJt, inertiaInvA, aJ);
		m_1MinvJt.set(0.0, 0.0, 0.0);
		Adiag = massInvA + m_0MinvJt.dot(aJ);

		assert (Adiag > 0.0);
	}

	public double getDiagonal() { return Adiag; }

	public double getRelativeVelocity(Vector3d linVelA, Vector3d angVelA, Vector3d linVelB, Vector3d angVelB) {
		Vector3d linRel = Stack.newVec();
		linRel.sub(linVelA, linVelB);

		Vector3d angVelAi = Stack.newVec();
		VectorUtil.mul(angVelAi, angVelA, aJ);

		Vector3d angVelBi = Stack.newVec();
		VectorUtil.mul(angVelBi, angVelB, bJ);

		VectorUtil.mul(linRel, linRel, linearJointAxis);

		angVelAi.add(angVelBi);
		angVelAi.add(linRel);

		double relVel2 = angVelAi.x + angVelAi.y + angVelAi.z;
		return relVel2 + BulletGlobals.FLT_EPSILON;
	}
}
