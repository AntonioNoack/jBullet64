package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * SolverBody is an internal data structure for the constraint solver. Only necessary
 * data is packed to increase cache coherence/performance.
 * 
 * @author jezek2
 */
public class SolverBody {
	
	//protected final BulletStack stack = BulletStack.get();

	public final Vector3d angularVelocity = new Vector3d();
	public double angularFactor;
	public double invMass;
	public double friction;
	public RigidBody originalBody;
	public final Vector3d linearVelocity = new Vector3d();
	public final Vector3d centerOfMassPosition = new Vector3d();

	public final Vector3d pushVelocity = new Vector3d();
	public final Vector3d turnVelocity = new Vector3d();
	
	public void getVelocityInLocalPoint(Vector3d rel_pos, Vector3d velocity) {
		Vector3d tmp = Stack.newVec();
		tmp.cross(angularVelocity, rel_pos);
		velocity.add(linearVelocity, tmp);
	}

	/**
	 * Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position.
	 */
	public void internalApplyImpulse(Vector3d linearComponent, Vector3d angularComponent, double impulseMagnitude) {
		if (invMass != 0.0) {
			linearVelocity.scaleAdd(impulseMagnitude, linearComponent, linearVelocity);
			angularVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
		}
	}

	public void internalApplyPushImpulse(Vector3d linearComponent, Vector3d angularComponent, double impulseMagnitude) {
		if (invMass != 0.0) {
			pushVelocity.scaleAdd(impulseMagnitude, linearComponent, pushVelocity);
			turnVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, turnVelocity);
		}
	}
	
	public void writebackVelocity() {
		if (invMass != 0.0) {
			originalBody.setLinearVelocity(linearVelocity);
			originalBody.setAngularVelocity(angularVelocity);
			//m_originalBody->setCompanionId(-1);
		}
	}

	public void writebackVelocity(double timeStep) {
		if (invMass != 0.0) {
			originalBody.setLinearVelocity(linearVelocity);
			originalBody.setAngularVelocity(angularVelocity);

			// correct the position/orientation based on push/turn recovery
			Transform newTransform = Stack.newTrans();
			Transform curTrans = originalBody.getWorldTransform(Stack.newTrans());
			TransformUtil.integrateTransform(curTrans, pushVelocity, turnVelocity, timeStep, newTransform);
			originalBody.setWorldTransform(newTransform);

			Stack.subTrans(2);
			//m_originalBody->setCompanionId(-1);
		}
	}
	
	public void readVelocity() {
		if (invMass != 0.0) {
			originalBody.getLinearVelocity(linearVelocity);
			originalBody.getAngularVelocity(angularVelocity);
		}
	}
	
}
