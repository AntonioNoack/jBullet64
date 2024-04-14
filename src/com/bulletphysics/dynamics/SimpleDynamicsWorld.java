package com.bulletphysics.dynamics;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo;
import com.bulletphysics.linearmath.Transform;
import java.util.ArrayList;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * SimpleDynamicsWorld serves as unit-test and to verify more complicated and
 * optimized dynamics worlds. Please use {@link DiscreteDynamicsWorld} instead
 * (or ContinuousDynamicsWorld once it is finished).
 * 
 * @author jezek2
 */
public class SimpleDynamicsWorld extends DynamicsWorld {

	protected ConstraintSolver constraintSolver;
	protected final Vector3d gravity = new Vector3d(0f, 0.0, -10f);
	
	public SimpleDynamicsWorld(Dispatcher dispatcher, BroadphaseInterface pairCache, ConstraintSolver constraintSolver, CollisionConfiguration collisionConfiguration) {
		super(dispatcher, pairCache, collisionConfiguration);
		this.constraintSolver = constraintSolver;
	}

	protected void predictUnconstraintMotion(double timeStep) {
		Transform tmpTrans = Stack.newTrans();

		for (CollisionObject colObj : collisionObjects) {
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				if (!body.isStaticObject()) {
					if (body.isActive()) {
						body.applyGravity();
						body.integrateVelocities(timeStep);
						body.applyDamping(timeStep);
						body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
					}
				}
			}
		}
	}
	
	protected void integrateTransforms(double timeStep) {
		Transform predictedTrans = Stack.newTrans();
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.get(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				if (body.isActive() && (!body.isStaticObject())) {
					body.predictIntegratedTransform(timeStep, predictedTrans);
					body.proceedToTransform(predictedTrans);
				}
			}
		}
		Stack.subTrans(1);
	}
	
	/**
	 * maxSubSteps/fixedTimeStep for interpolation is currently ignored for SimpleDynamicsWorld, use DiscreteDynamicsWorld instead.
	 */
	@Override
	public int stepSimulation(double timeStep, int maxSubSteps, double fixedTimeStep) {
		// apply gravity, predict motion
		predictUnconstraintMotion(timeStep);

		DispatcherInfo dispatchInfo = getDispatchInfo();
		dispatchInfo.timeStep = timeStep;
		dispatchInfo.stepCount = 0;
		dispatchInfo.debugDraw = getDebugDrawer();

		// perform collision detection
		performDiscreteCollisionDetection();

		// solve contact constraints
		int numManifolds = dispatcher1.getNumManifolds();
		if (numManifolds != 0)
		{
			ArrayList<PersistentManifold> manifoldPtr = ((CollisionDispatcher)dispatcher1).getInternalManifoldPointer();

			ContactSolverInfo infoGlobal = new ContactSolverInfo();
			infoGlobal.timeStep = timeStep;
			constraintSolver.prepareSolve(0,numManifolds);
			constraintSolver.solveGroup(null,0,manifoldPtr, 0, numManifolds, null,0,0,infoGlobal,debugDrawer/*, m_stackAlloc*/,dispatcher1);
			constraintSolver.allSolved(infoGlobal,debugDrawer/*, m_stackAlloc*/);
		}

		// integrate transforms
		integrateTransforms(timeStep);

		updateAabbs();

		synchronizeMotionStates();

		clearForces();

		return 1;
	}

	@Override
	public void clearForces() {
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.get(i);

			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				body.clearForces();
			}
		}
	}

	@Override
	public void setGravity(Vector3d gravity) {
		this.gravity.set(gravity);
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.get(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				body.setGravity(gravity);
			}
		}
	}

	@Override
	public Vector3d getGravity(Vector3d out) {
		out.set(gravity);
		return out;
	}

	@Override
	public void addRigidBody(RigidBody body) {
		body.setGravity(gravity);

		if (body.getCollisionShape() != null) {
			addCollisionObject(body);
		}
	}

	@Override
	public void removeRigidBody(RigidBody body) {
		removeCollisionObject(body);
	}

	@Override
	public void updateAabbs() {
		Transform tmpTrans = Stack.newTrans();
		// Transform predictedTrans = Stack.newTrans();
		Vector3d minAabb = Stack.newVec(), maxAabb = Stack.newVec();

		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.get(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				if (body.isActive() && (!body.isStaticObject())) {
					colObj.getCollisionShape().getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
					BroadphaseInterface bp = getBroadphase();
					bp.setAabb(body.getBroadphaseHandle(), minAabb, maxAabb, dispatcher1);
				}
			}
		}
	}

	public void synchronizeMotionStates() {
		Transform tmpTrans = Stack.newTrans();
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size(); i++) {
			CollisionObject colObj = collisionObjects.get(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null && body.getMotionState() != null) {
				if (body.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
					body.getMotionState().setWorldTransform(body.getWorldTransform(tmpTrans));
				}
			}
		}
		Stack.subTrans(1);
	}

	@Override
	public void setConstraintSolver(ConstraintSolver solver) {
		constraintSolver = solver;
	}

	@Override
	public ConstraintSolver getConstraintSolver() {
		return constraintSolver;
	}
	
	@Override
	public void debugDrawWorld() {
		// TODO: throw new UnsupportedOperationException("Not supported yet.");
	}

}
