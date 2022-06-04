package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.linearmath.IDebugDraw;
import java.util.ArrayList;

/**
 * Abstract class for constraint solvers.
 *
 * @author jezek2
 */
public abstract class ConstraintSolver {

    //protected final BulletStack stack = BulletStack.get();

    public void prepareSolve(int numBodies, int numManifolds) {
    }

    /**
     * Solve a group of constraints.
     */
    @SuppressWarnings("UnusedReturnValue")
    public abstract double solveGroup(ArrayList<CollisionObject> bodies, int numBodies, ArrayList<PersistentManifold> manifold, int manifold_offset, int numManifolds, ArrayList<TypedConstraint> constraints, int constraints_offset, int numConstraints, ContactSolverInfo info, IDebugDraw debugDrawer/*, btStackAlloc* stackAlloc*/, Dispatcher dispatcher);

    public void allSolved(ContactSolverInfo info, IDebugDraw debugDrawer/*, btStackAlloc* stackAlloc*/) {
    }

    /**
     * Clear internal cached data and reset random seed.
     */
    public abstract void reset();

}
