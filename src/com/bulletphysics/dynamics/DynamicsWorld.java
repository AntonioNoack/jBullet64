package com.bulletphysics.dynamics;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.dynamics.constraintsolver.BrokenConstraintCallback;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.vehicle.RaycastVehicle;

import javax.vecmath.Vector3d;

/**
 * DynamicsWorld is the interface class for several dynamics implementation,
 * basic, discrete, parallel, and continuous etc.
 *
 * @author jezek2
 */
public abstract class DynamicsWorld extends CollisionWorld {

    public BrokenConstraintCallback brokenConstraintCallback;
    public InternalTickCallback internalTickCallback;
    public Object worldUserInfo;

    public final ContactSolverInfo solverInfo = new ContactSolverInfo();

    public DynamicsWorld(Dispatcher dispatcher, BroadphaseInterface broadphasePairCache, CollisionConfiguration collisionConfiguration) {
        super(dispatcher, broadphasePairCache, collisionConfiguration);
    }

    @SuppressWarnings("UnusedReturnValue")
    public final int stepSimulation(double timeStep) {
        return stepSimulation(timeStep, 1, 1.0 / 60f);
    }

    @SuppressWarnings("UnusedReturnValue")
    public final int stepSimulation(double timeStep, int maxSubSteps) {
        return stepSimulation(timeStep, maxSubSteps, 1.0 / 60f);
    }

    /**
     * Proceeds the simulation over 'timeStep', units in preferably in seconds.<p>
     * <p>
     * By default, Bullet will subdivide the timestep in constant substeps of each
     * 'fixedTimeStep'.<p>
     * <p>
     * In order to keep the simulation real-time, the maximum number of substeps can
     * be clamped to 'maxSubSteps'.<p>
     * <p>
     * You can disable subdividing the timestep/substepping by passing maxSubSteps=0
     * as second argument to stepSimulation, but in that case you have to keep the
     * timeStep constant.
     */
    public abstract int stepSimulation(double timeStep, int maxSubSteps, double fixedTimeStep);

    @SuppressWarnings("unused")
    public abstract void debugDrawWorld();

    public final void addConstraint(TypedConstraint constraint) {
        addConstraint(constraint, false);
    }

    public void addConstraint(TypedConstraint constraint, boolean disableCollisionsBetweenLinkedBodies) {
    }

    @SuppressWarnings("unused")
    public void removeConstraint(TypedConstraint constraint) {
    }

    @SuppressWarnings("unused")
    public void addAction(ActionInterface action) {
    }

    @SuppressWarnings("unused")
    public void removeAction(ActionInterface action) {
    }

    public void addVehicle(RaycastVehicle vehicle) {
    }

    @SuppressWarnings("unused")
    public void removeVehicle(RaycastVehicle vehicle) {
    }

    /**
     * Once a rigidbody is added to the dynamics world, it will get this gravity assigned.
     * Existing rigidbodies in the world get gravity assigned too, during this method.
     */
    public abstract void setGravity(Vector3d gravity);

    @SuppressWarnings("unused")
    public abstract Vector3d getGravity(Vector3d out);

    public abstract void addRigidBody(RigidBody body);

    @SuppressWarnings("unused")
    public abstract void removeRigidBody(RigidBody body);

    @SuppressWarnings("unused")
    public abstract void setConstraintSolver(ConstraintSolver solver);

    @SuppressWarnings("unused")
    public abstract ConstraintSolver getConstraintSolver();

    public int getNumConstraints() {
        return 0;
    }

    @SuppressWarnings("unused")
    public TypedConstraint getConstraint(int index) {
        return null;
    }

    @SuppressWarnings("unused")
    public int getNumActions() {
        return 0;
    }

    @SuppressWarnings("unused")
    public ActionInterface getAction(int index) {
        return null;
    }

    public abstract void clearForces();
}
