package com.bulletphysics.dynamics.constraintsolver

import com.bulletphysics.collision.broadphase.Dispatcher
import com.bulletphysics.collision.dispatch.CollisionObject
import com.bulletphysics.collision.narrowphase.PersistentManifold
import com.bulletphysics.linearmath.IDebugDraw
import com.bulletphysics.util.ObjectArrayList

/**
 * Abstract class for constraint solvers.
 *
 * @author jezek2
 */
abstract class ConstraintSolver {
    fun prepareSolve(numBodies: Int, numManifolds: Int) {
    }

    /**
     * Solve a group of constraints.
     */
    abstract fun solveGroup(
        bodies: ObjectArrayList<CollisionObject>, numBodies: Int,
        manifold: ObjectArrayList<PersistentManifold>, manifoldOffset: Int, numManifolds: Int,
        constraints: ObjectArrayList<TypedConstraint>, constraintsOffset: Int, numConstraints: Int,
        info: ContactSolverInfo, debugDrawer: IDebugDraw?, dispatcher: Dispatcher
    ): Double

    fun allSolved(info: ContactSolverInfo?, debugDrawer: IDebugDraw? /*, btStackAlloc* stackAlloc*/) {
    }

    /**
     * Clear internal cached data and reset random seed.
     */
    abstract fun reset()
}
