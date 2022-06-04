package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;
import cz.advel.stack.StaticAlloc;
import javax.vecmath.Vector3d;

/**
 * GjkPairDetector uses GJK to implement the {@link DiscreteCollisionDetectorInterface}.
 * 
 * @author jezek2
 */
public class GjkPairDetector extends DiscreteCollisionDetectorInterface {

	//protected final BulletStack stack = BulletStack.get();
	
	// must be above the machine epsilon
	private static final double REL_ERROR2 = 1.0e-6f;
	
	private final Vector3d cachedSeparatingAxis = new Vector3d();
	private ConvexPenetrationDepthSolver penetrationDepthSolver;
	private SimplexSolverInterface simplexSolver;
	private ConvexShape minkowskiA;
	private ConvexShape minkowskiB;
	private boolean ignoreMargin;
	
	// some debugging to fix degeneracy problems
	public int lastUsedMethod;
	public int curIter;
	public int degenerateSimplex;
	public int catchDegeneracies;
	
	public void init(ConvexShape objectA, ConvexShape objectB, SimplexSolverInterface simplexSolver, ConvexPenetrationDepthSolver penetrationDepthSolver) {
		this.cachedSeparatingAxis.set(0.0, 0.0, 1.0);
		this.ignoreMargin = false;
		this.lastUsedMethod = -1;
		this.catchDegeneracies = 1;
		
		this.penetrationDepthSolver = penetrationDepthSolver;
		this.simplexSolver = simplexSolver;
		this.minkowskiA = objectA;
		this.minkowskiB = objectB;
	}
	
	@StaticAlloc
	public void getClosestPoints(ClosestPointInput input, Result output, IDebugDraw debugDraw, boolean swapResults) {

		int v3 = Stack.getVecPosition();
		int t3 = Stack.getTransPosition();

		Vector3d tmp = Stack.newVec();

		double distance = 0.0;
		Vector3d normalInB = Stack.newVec();
		normalInB.set(0.0, 0.0, 0.0);
		Vector3d pointOnA = Stack.newVec(), pointOnB = Stack.newVec();
		Transform localTransA = Stack.newTrans(input.transformA);
		Transform localTransB = Stack.newTrans(input.transformB);
		Vector3d positionOffset = Stack.newVec();
		positionOffset.add(localTransA.origin, localTransB.origin);
		positionOffset.scale(0.5);
		localTransA.origin.sub(positionOffset);
		localTransB.origin.sub(positionOffset);

		double marginA = minkowskiA.getMargin();
		double marginB = minkowskiB.getMargin();

		BulletStats.gNumGjkChecks++;

		// for CCD we don't use margins
		if (ignoreMargin) {
			marginA = 0.0;
			marginB = 0.0;
		}

		curIter = 0;
		int gGjkMaxIter = 1000; // this is to catch invalid input, perhaps check for #NaN?
		cachedSeparatingAxis.set(0.0, 1.0, 0.0);

		boolean isValid = false;
		boolean checkSimplex = false;
		boolean checkPenetration = true;
		degenerateSimplex = 0;

		lastUsedMethod = -1;

		{
			double squaredDistance = BulletGlobals.SIMD_INFINITY;
			double delta;

			double margin = marginA + marginB;

			simplexSolver.reset();

			Vector3d separatingAxisInA = Stack.newVec();
			Vector3d separatingAxisInB = Stack.newVec();
			
			Vector3d pInA = Stack.newVec();
			Vector3d qInB = Stack.newVec();
			
			Vector3d pWorld = Stack.newVec();
			Vector3d qWorld = Stack.newVec();
			Vector3d w = Stack.newVec();
			
			Vector3d tmpPointOnA = Stack.newVec(), tmpPointOnB = Stack.newVec();
			Vector3d tmpNormalInB = Stack.newVec();
			
			for (;;) //while (true)
			{
				separatingAxisInA.negate(cachedSeparatingAxis);
				MatrixUtil.transposeTransform(separatingAxisInA, separatingAxisInA, input.transformA.basis);

				separatingAxisInB.set(cachedSeparatingAxis);
				MatrixUtil.transposeTransform(separatingAxisInB, separatingAxisInB, input.transformB.basis);

				minkowskiA.localGetSupportingVertexWithoutMargin(separatingAxisInA, pInA);
				minkowskiB.localGetSupportingVertexWithoutMargin(separatingAxisInB, qInB);

				pWorld.set(pInA);
				localTransA.transform(pWorld);
				
				qWorld.set(qInB);
				localTransB.transform(qWorld);

				w.sub(pWorld, qWorld);

				delta = cachedSeparatingAxis.dot(w);

				// potential exit, they don't overlap
				if ((delta > 0.0) && (delta * delta > squaredDistance * input.maximumDistanceSquared)) {
					checkPenetration = false;
					break;
				}

				// exit 0: the new point is already in the simplex, or we didn't come any closer
				if (simplexSolver.inSimplex(w)) {
					degenerateSimplex = 1;
					checkSimplex = true;
					break;
				}
				// are we getting any closer ?
				double f0 = squaredDistance - delta;
				double f1 = squaredDistance * REL_ERROR2;

				if (f0 <= f1) {
					if (f0 <= 0.0) {
						degenerateSimplex = 2;
					}
					checkSimplex = true;
					break;
				}
				// add current vertex to simplex
				simplexSolver.addVertex(w, pWorld, qWorld);

				// calculate the closest point to the origin (update vector v)
				if (!simplexSolver.closest(cachedSeparatingAxis)) {
					degenerateSimplex = 3;
					checkSimplex = true;
					break;
				}

				if (cachedSeparatingAxis.lengthSquared() < REL_ERROR2) {
					degenerateSimplex = 6;
					checkSimplex = true;
					break;
				}
				
				double previousSquaredDistance = squaredDistance;
				squaredDistance = cachedSeparatingAxis.lengthSquared();

				// redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

				// are we getting any closer ?
				if (previousSquaredDistance - squaredDistance <= BulletGlobals.FLT_EPSILON * previousSquaredDistance) {
					simplexSolver.backup_closest(cachedSeparatingAxis);
					checkSimplex = true;
					break;
				}

				// degeneracy, this is typically due to invalid/uninitialized worldtransforms for a CollisionObject   
				if (curIter++ > gGjkMaxIter) {
					//#if defined(DEBUG) || defined (_DEBUG)   
					if (BulletGlobals.DEBUG) {
						System.err.printf("btGjkPairDetector maxIterations exceeded:%d\n", curIter);
						System.err.printf("sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%s,shapeTypeB=%s\n",
								cachedSeparatingAxis.x,
								cachedSeparatingAxis.y,
								cachedSeparatingAxis.z,
								squaredDistance,
								minkowskiA.getShapeType().name(),
								minkowskiB.getShapeType().name());
					}
					//#endif   
					break;

				}

				boolean check = (!simplexSolver.fullSimplex());
				//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());

				if (!check) {
					// do we need this backup_closest here ?
					simplexSolver.backup_closest(cachedSeparatingAxis);
					break;
				}
			}

			if (checkSimplex) {
				simplexSolver.compute_points(pointOnA, pointOnB);
				normalInB.sub(pointOnA, pointOnB);
				double lenSqr = cachedSeparatingAxis.lengthSquared();
				// valid normal
				if (lenSqr < 0.0001f) {
					degenerateSimplex = 5;
				}
				if (lenSqr > BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON) {
					double rlen = 1.0 / Math.sqrt(lenSqr);
					normalInB.scale(rlen); // normalize
					double s = Math.sqrt(squaredDistance);

					assert (s > 0.0);

					tmp.scale((marginA / s), cachedSeparatingAxis);
					pointOnA.sub(tmp);

					tmp.scale((marginB / s), cachedSeparatingAxis);
					pointOnB.add(tmp);

					distance = ((1.0 / rlen) - margin);
					isValid = true;

					lastUsedMethod = 1;
				}
				else {
					lastUsedMethod = 2;
				}
			}

			boolean catchDegeneratePenetrationCase =
					(catchDegeneracies != 0 && penetrationDepthSolver != null && degenerateSimplex != 0 && ((distance + margin) < 0.01f));

			//if (checkPenetration && !isValid)
			if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
				// penetration case

				// if there is no way to handle penetrations, bail out
				if (penetrationDepthSolver != null) {
					// Penetration depth case.
					BulletStats.gNumDeepPenetrationChecks++;

					boolean isValid2 = penetrationDepthSolver.calcPenDepth(
							simplexSolver,
							minkowskiA, minkowskiB,
							localTransA, localTransB,
							cachedSeparatingAxis, tmpPointOnA, tmpPointOnB,
							debugDraw/*,input.stackAlloc*/);

					if (isValid2) {
						tmpNormalInB.sub(tmpPointOnB, tmpPointOnA);

						double lenSqr = tmpNormalInB.lengthSquared();
						if (lenSqr > (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
							tmpNormalInB.scale(1.0 / Math.sqrt(lenSqr));
							tmp.sub(tmpPointOnA, tmpPointOnB);
							double distance2 = -tmp.length();
							// only replace valid penetrations when the result is deeper (check)
							if (!isValid || (distance2 < distance)) {
								distance = distance2;
								pointOnA.set(tmpPointOnA);
								pointOnB.set(tmpPointOnB);
								normalInB.set(tmpNormalInB);
								isValid = true;
								lastUsedMethod = 3;
							}
						}
						else {
							//isValid = false;
							lastUsedMethod = 4;
						}
					}
					else {
						lastUsedMethod = 5;
					}

				}
			}
		}

		if (isValid) {
			//#ifdef __SPU__
			//		//spu_printf("distance\n");
			//#endif //__CELLOS_LV2__

			tmp.add(pointOnB, positionOffset);
			output.addContactPoint(
					normalInB,
					tmp,
					distance);
		//printf("gjk add:%f",distance);
		}

		Stack.resetVec(v3);
		Stack.resetTrans(t3);

	}

	public void setMinkowskiA(ConvexShape minkA) {
		minkowskiA = minkA;
	}

	public void setMinkowskiB(ConvexShape minkB) {
		minkowskiB = minkB;
	}

	public void setCachedSeparatingAxis(Vector3d separatingAxis) {
		cachedSeparatingAxis.set(separatingAxis);
	}

	public void setPenetrationDepthSolver(ConvexPenetrationDepthSolver penetrationDepthSolver) {
		this.penetrationDepthSolver = penetrationDepthSolver;
	}

	/**
	 * Don't use setIgnoreMargin, it's for Bullet's internal use.
	 */
	public void setIgnoreMargin(boolean ignoreMargin) {
		this.ignoreMargin = ignoreMargin;
	}
	
}
