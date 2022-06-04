package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.narrowphase.SubSimplexConvexCast;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import java.util.ArrayList;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ConvexConcaveCollisionAlgorithm supports collision between convex shapes
 * and (concave) triangle meshes.
 *
 * @author jezek2
 */
public class ConvexConcaveCollisionAlgorithm extends CollisionAlgorithm {

    private boolean isSwapped;
    private ConvexTriangleCallback btConvexTriangleCallback;

    public void init(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, boolean isSwapped) {
        super.init(ci);
        this.isSwapped = isSwapped;
        this.btConvexTriangleCallback = new ConvexTriangleCallback(dispatcher, body0, body1, isSwapped);
    }
	
    @Override
    public void destroy() {
        btConvexTriangleCallback.destroy();
    }

    @Override
    public void processCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        CollisionObject convexBody = isSwapped ? body1 : body0;
        CollisionObject triBody = isSwapped ? body0 : body1;

        if (triBody.getCollisionShape().isConcave()) {
            ConcaveShape concaveShape = (ConcaveShape) triBody.getCollisionShape();

            if (convexBody.getCollisionShape().isConvex()) {
                double collisionMarginTriangle = concaveShape.getMargin();

                resultOut.setPersistentManifold(btConvexTriangleCallback.manifoldPtr);
                btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, resultOut);

                // Disable persistency. previously, some older algorithm calculated all contacts in one go, so you can clear it here.
                //m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);

                btConvexTriangleCallback.manifoldPtr.setBodies(convexBody, triBody);

                concaveShape.processAllTriangles(
                        btConvexTriangleCallback,
                        btConvexTriangleCallback.getAabbMin(Stack.newVec()),
                        btConvexTriangleCallback.getAabbMax(Stack.newVec()));

                resultOut.refreshContactPoints();
            }
        }
    }

    @Override
    public double calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        Vector3d tmp = Stack.newVec();

        CollisionObject convexBody = isSwapped ? body1 : body0;
        CollisionObject triBody = isSwapped ? body0 : body1;

        // quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

        // only perform CCD above a certain threshold, this prevents blocking on the long run
        // because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
        tmp.sub(convexBody.getInterpolationWorldTransform(Stack.newTrans()).origin, convexBody.getWorldTransform(Stack.newTrans()).origin);
        double squareMot0 = tmp.lengthSquared();
        if (squareMot0 < convexBody.getCcdSquareMotionThreshold()) {
            return 1.0;
        }

        Transform tmpTrans = Stack.newTrans();

        //const btVector3& from = convexbody->m_worldTransform.getOrigin();
        //btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
        //todo: only do if the motion exceeds the 'radius'

        Transform triInv = triBody.getWorldTransform(Stack.newTrans());
        triInv.inverse();

        Transform convexFromLocal = Stack.newTrans();
        convexFromLocal.mul(triInv, convexBody.getWorldTransform(tmpTrans));

        Transform convexToLocal = Stack.newTrans();
        convexToLocal.mul(triInv, convexBody.getInterpolationWorldTransform(tmpTrans));

        if (triBody.getCollisionShape().isConcave()) {
            Vector3d rayAabbMin = Stack.newVec(convexFromLocal.origin);
            VectorUtil.setMin(rayAabbMin, convexToLocal.origin);

            Vector3d rayAabbMax = Stack.newVec(convexFromLocal.origin);
            VectorUtil.setMax(rayAabbMax, convexToLocal.origin);

            double ccdRadius0 = convexBody.getCcdSweptSphereRadius();

            tmp.set(ccdRadius0, ccdRadius0, ccdRadius0);
            rayAabbMin.sub(tmp);
            rayAabbMax.add(tmp);

            double curHitFraction = 1.0; // is this available?
            LocalTriangleSphereCastCallback raycastCallback = new LocalTriangleSphereCastCallback(convexFromLocal, convexToLocal, convexBody.getCcdSweptSphereRadius(), curHitFraction);

            raycastCallback.hitFraction = convexBody.getHitFraction();

            ConcaveShape triangleMesh = (ConcaveShape) triBody.getCollisionShape();

            if (triangleMesh != null) {
                triangleMesh.processAllTriangles(raycastCallback, rayAabbMin, rayAabbMax);
            }

            if (raycastCallback.hitFraction < convexBody.getHitFraction()) {
                convexBody.setHitFraction(raycastCallback.hitFraction);
                return raycastCallback.hitFraction;
            }
        }

        return 1.0;
    }

    @Override
    public void getAllContactManifolds(ArrayList<PersistentManifold> manifoldArray) {
        if (btConvexTriangleCallback.manifoldPtr != null) {
            manifoldArray.add(btConvexTriangleCallback.manifoldPtr);
        }
    }

    public void clearCache() {
        btConvexTriangleCallback.clearCache();
    }

    ////////////////////////////////////////////////////////////////////////////

    private static class LocalTriangleSphereCastCallback extends TriangleCallback {
        public final Transform ccdSphereFromTrans = new Transform();
        public final Transform ccdSphereToTrans = new Transform();
        public final Transform meshTransform = new Transform();

        public double ccdSphereRadius;
        public double hitFraction;

        private final Transform identity = new Transform();

        public LocalTriangleSphereCastCallback(Transform from, Transform to, double ccdSphereRadius, double hitFraction) {
            this.ccdSphereFromTrans.set(from);
            this.ccdSphereToTrans.set(to);
            this.ccdSphereRadius = ccdSphereRadius;
            this.hitFraction = hitFraction;

            // JAVA NOTE: moved here from processTriangle
            identity.setIdentity();
        }

        public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
            // do a swept sphere for now

            //btTransform ident;
            //ident.setIdentity();

            CastResult castResult = new CastResult();
            castResult.fraction = hitFraction;
            SphereShape pointShape = new SphereShape(ccdSphereRadius);
            TriangleShape triShape = new TriangleShape(triangle[0], triangle[1], triangle[2]);
            VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
            SubSimplexConvexCast convexCaster = new SubSimplexConvexCast(pointShape, triShape, simplexSolver);
            //GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
            //ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
            //local space?

            if (convexCaster.calcTimeOfImpact(ccdSphereFromTrans, ccdSphereToTrans, identity, identity, castResult)) {
                if (hitFraction > castResult.fraction) {
                    hitFraction = castResult.fraction;
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class CreateFunc extends CollisionAlgorithmCreateFunc {
        private final ObjectPool<ConvexConcaveCollisionAlgorithm> pool = ObjectPool.get(ConvexConcaveCollisionAlgorithm.class);

        @Override
        public CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
            ConvexConcaveCollisionAlgorithm algo = pool.get();
            algo.init(ci, body0, body1, false);
            return algo;
        }

        @Override
        public void releaseCollisionAlgorithm(CollisionAlgorithm algo) {
            pool.release((ConvexConcaveCollisionAlgorithm) algo);
        }
    }

    public static class SwappedCreateFunc extends CollisionAlgorithmCreateFunc {
        private final ObjectPool<ConvexConcaveCollisionAlgorithm> pool = ObjectPool.get(ConvexConcaveCollisionAlgorithm.class);

        @Override
        public CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
            ConvexConcaveCollisionAlgorithm algo = pool.get();
            algo.init(ci, body0, body1, true);
            return algo;
        }

        @Override
        public void releaseCollisionAlgorithm(CollisionAlgorithm algo) {
            pool.release((ConvexConcaveCollisionAlgorithm) algo);
        }
    }

}
