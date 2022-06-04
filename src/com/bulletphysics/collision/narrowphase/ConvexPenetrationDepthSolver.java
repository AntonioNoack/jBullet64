package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import javax.vecmath.Vector3d;

/**
 * ConvexPenetrationDepthSolver provides an interface for penetration depth calculation.
 * 
 * @author jezek2
 */
public abstract class ConvexPenetrationDepthSolver {

	public abstract boolean calcPenDepth(SimplexSolverInterface simplexSolver,
			ConvexShape convexA, ConvexShape convexB,
			Transform transA, Transform transB,
			Vector3d v, Vector3d pa, Vector3d pb,
			IDebugDraw debugDraw/*, btStackAlloc* stackAlloc*/);
	
}
