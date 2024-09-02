package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ConvexPlaneCollisionAlgorithm provides convex/plane collision detection.
 *
 * @author jezek2
 */
public class ConvexPlaneCollisionAlgorithm extends CollisionAlgorithm {

    private boolean ownManifold;
    private PersistentManifold manifoldPtr;
    private boolean isSwapped;

    public void init(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1, boolean isSwapped) {
        super.init(ci);
        this.ownManifold = false;
        this.manifoldPtr = mf;
        this.isSwapped = isSwapped;

        CollisionObject convexObj = isSwapped ? col1 : col0;
        CollisionObject planeObj = isSwapped ? col0 : col1;

        if (manifoldPtr == null && dispatcher.needsCollision(convexObj, planeObj)) {
            manifoldPtr = dispatcher.getNewManifold(convexObj, planeObj);
            ownManifold = true;
        }
    }

    @Override
    public void destroy() {
        if (ownManifold) {
            if (manifoldPtr != null) {
                dispatcher.releaseManifold(manifoldPtr);
            }
            manifoldPtr = null;
        }
    }

    @Override
    public void processCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        if (manifoldPtr == null) {
            return;
        }

        Transform tmpTrans = Stack.newTrans();

        CollisionObject convexObj = isSwapped ? body1 : body0;
        CollisionObject planeObj = isSwapped ? body0 : body1;

        ConvexShape convexShape = (ConvexShape) convexObj.getCollisionShape();
        StaticPlaneShape planeShape = (StaticPlaneShape) planeObj.getCollisionShape();

        boolean hasCollision = false;
        Vector3d planeNormal = planeShape.getPlaneNormal(Stack.newVec());
        double planeConstant = planeShape.getPlaneConstant();

        Transform planeInConvex = Stack.newTrans();
        convexObj.getWorldTransform(planeInConvex);
        planeInConvex.inverse();
        planeInConvex.mul(planeObj.getWorldTransform(tmpTrans));

        Transform convexInPlaneTrans = Stack.newTrans();
        convexInPlaneTrans.inverse(planeObj.getWorldTransform(tmpTrans));
        convexInPlaneTrans.mul(convexObj.getWorldTransform(tmpTrans));

        Vector3d tmp = Stack.newVec();
        tmp.negate(planeNormal);
        planeInConvex.basis.transform(tmp);

        Vector3d vtx = convexShape.localGetSupportingVertex(tmp, Stack.newVec());
        Vector3d vtxInPlane = Stack.newVec(vtx);
        convexInPlaneTrans.transform(vtxInPlane);

        double distance = (planeNormal.dot(vtxInPlane) - planeConstant);

        Vector3d vtxInPlaneProjected = Stack.newVec();
        tmp.scale(distance, planeNormal);
        vtxInPlaneProjected.sub(vtxInPlane, tmp);

        Vector3d vtxInPlaneWorld = Stack.newVec(vtxInPlaneProjected);
        planeObj.getWorldTransform(tmpTrans).transform(vtxInPlaneWorld);

        hasCollision = distance < manifoldPtr.getContactBreakingThreshold();
        resultOut.setPersistentManifold(manifoldPtr);
        if (hasCollision) {
            // report a contact. internally this will be kept persistent, and contact reduction is done
            Vector3d normalOnSurfaceB = Stack.newVec(planeNormal);
            planeObj.getWorldTransform(tmpTrans).basis.transform(normalOnSurfaceB);

            Vector3d pOnB = Stack.newVec(vtxInPlaneWorld);
            resultOut.addContactPoint(normalOnSurfaceB, pOnB, distance);
        }
        if (ownManifold) {
            if (manifoldPtr.getNumContacts() != 0) {
                resultOut.refreshContactPoints();
            }
        }
    }

    @Override
    public double calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        // not yet
        return 1.0;
    }

    @Override
    public void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray) {
        if (manifoldPtr != null && ownManifold) {
            manifoldArray.add(manifoldPtr);
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class CreateFunc extends CollisionAlgorithmCreateFunc {
        private final ObjectPool<ConvexPlaneCollisionAlgorithm> pool = ObjectPool.get(ConvexPlaneCollisionAlgorithm.class);

        @Override
        public CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
            ConvexPlaneCollisionAlgorithm algo = pool.get();
            algo.init(null, ci, body0, body1, swapped);
            return algo;
        }

        @Override
        public void releaseCollisionAlgorithm(CollisionAlgorithm algo) {
            pool.release((ConvexPlaneCollisionAlgorithm) algo);
        }
    }

}
