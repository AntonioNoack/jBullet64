package com.bulletphysics.collision.dispatch;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * Provides collision detection between two spheres.
 *
 * @author jezek2
 */
public class SphereSphereCollisionAlgorithm extends CollisionAlgorithm {

    private boolean ownManifold;
    private PersistentManifold manifoldPtr;

    public void init(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1) {
        super.init(ci);
        manifoldPtr = mf;

        if (manifoldPtr == null) {
            manifoldPtr = dispatcher.getNewManifold(col0, col1);
            ownManifold = true;
        }
    }

    @Override
    public void init(CollisionAlgorithmConstructionInfo ci) {
        super.init(ci);
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
    public void processCollision(CollisionObject col0, CollisionObject col1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        if (manifoldPtr == null) {
            return;
        }

        Transform tmpTrans1 = Stack.newTrans();
        Transform tmpTrans2 = Stack.newTrans();

        resultOut.setPersistentManifold(manifoldPtr);

        SphereShape sphere0 = (SphereShape) col0.getCollisionShape();
        SphereShape sphere1 = (SphereShape) col1.getCollisionShape();

        Vector3d diff = Stack.newVec();
        diff.sub(col0.getWorldTransform(tmpTrans1).origin, col1.getWorldTransform(tmpTrans2).origin);

        double len = diff.length();
        double radius0 = sphere0.getRadius();
        double radius1 = sphere1.getRadius();

        //#ifdef CLEAR_MANIFOLD
        //manifoldPtr.clearManifold(); // don't do this, it disables warmstarting
        //#endif

        // if distance positive, don't generate a new contact
        if (len > (radius0 + radius1)) {
            //#ifndef CLEAR_MANIFOLD
            resultOut.refreshContactPoints();
            //#endif //CLEAR_MANIFOLD
            Stack.subVec(1);
            Stack.subTrans(2);
            return;
        }
        // distance (negative means penetration)
        double dist = len - (radius0 + radius1);

        Vector3d normalOnSurfaceB = Stack.newVec();
        normalOnSurfaceB.set(1.0, 0.0, 0.0);
        if (len > BulletGlobals.FLT_EPSILON) {
            normalOnSurfaceB.scale(1.0 / len, diff);
        }

        Vector3d tmp = Stack.newVec();

        // point on A (worldspace)
        Vector3d pos0 = Stack.newVec();
        tmp.scale(radius0, normalOnSurfaceB);
        pos0.sub(col0.getWorldTransform(tmpTrans1).origin, tmp);

        // point on B (worldspace)
        Vector3d pos1 = Stack.newVec();
        tmp.scale(radius1, normalOnSurfaceB);
        pos1.add(col1.getWorldTransform(tmpTrans2).origin, tmp);

        // report a contact. internally this will be kept persistent, and contact reduction is done
        resultOut.addContactPoint(normalOnSurfaceB, pos1, dist);

        //#ifndef CLEAR_MANIFOLD
        resultOut.refreshContactPoints();
        //#endif //CLEAR_MANIFOLD

        Stack.subVec(5);
        Stack.subTrans(2);

    }

    @Override
    public double calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
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
        private final ObjectPool<SphereSphereCollisionAlgorithm> pool = ObjectPool.get(SphereSphereCollisionAlgorithm.class);

        @Override
        public CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
            SphereSphereCollisionAlgorithm algo = pool.get();
            algo.init(null, ci, body0, body1);
            return algo;
        }

        @Override
        public void releaseCollisionAlgorithm(CollisionAlgorithm algo) {
            pool.release((SphereSphereCollisionAlgorithm) algo);
        }
    }

    ;

}
