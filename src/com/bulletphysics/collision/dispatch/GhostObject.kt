package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * GhostObject can keep track of all objects that are overlapping. By default, this
 * overlap is based on the AABB. This is useful for creating a character controller,
 * collision sensors/triggers, explosions etc.
 *
 * @author tomrbryn
 */
public class GhostObject extends CollisionObject {

    protected ObjectArrayList<CollisionObject> overlappingObjects = new ObjectArrayList<>();

    /**
     * This method is mainly for expert/internal use only.
     */
    public void addOverlappingObjectInternal(BroadphaseProxy otherProxy, BroadphaseProxy thisProxy) {
        CollisionObject otherObject = (CollisionObject) otherProxy.clientObject;
        assert (otherObject != null);

        // if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
        int index = overlappingObjects.indexOf(otherObject);
        if (index == -1) {
            // not found
            overlappingObjects.add(otherObject);
        }
    }

    /**
     * This method is mainly for expert/internal use only.
     */
    public void removeOverlappingObjectInternal(BroadphaseProxy otherProxy, Dispatcher dispatcher, BroadphaseProxy thisProxy) {
        CollisionObject otherObject = (CollisionObject) otherProxy.clientObject;
        assert (otherObject != null);

        int index = overlappingObjects.indexOf(otherObject);
        if (index != -1) {
            overlappingObjects.set(index, overlappingObjects.getQuick(overlappingObjects.getSize() - 1));
            overlappingObjects.removeQuick(overlappingObjects.getSize() - 1);
        }
    }

    public void convexSweepTest(ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld, CollisionWorld.ConvexResultCallback resultCallback, double allowedCcdPenetration) {
        Transform convexFromTrans = Stack.newTrans();
        Transform convexToTrans = Stack.newTrans();

        convexFromTrans.set(convexFromWorld);
        convexToTrans.set(convexToWorld);

        Vector3d castShapeAabbMin = Stack.newVec();
        Vector3d castShapeAabbMax = Stack.newVec();

        // compute AABB that encompasses angular movement
        {
            Vector3d linVel = Stack.newVec();
            Vector3d angVel = Stack.newVec();
            TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1.0, linVel, angVel);
            Transform R = Stack.newTrans();
            R.setIdentity();
            R.setRotation(convexFromTrans.getRotation(Stack.newQuat()));
            castShape.calculateTemporalAabb(R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
        }

        Transform tmpTrans = Stack.newTrans();

        // go over all objects, and if the ray intersects their aabb + cast shape aabb,
        // do a ray-shape query using convexCaster (CCD)
        for (int i = 0; i < overlappingObjects.getSize(); i++) {
            CollisionObject collisionObject = overlappingObjects.getQuick(i);

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.broadphaseHandle)) {
                //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
                Vector3d collisionObjectAabbMin = Stack.newVec();
                Vector3d collisionObjectAabbMax = Stack.newVec();
                collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(tmpTrans), collisionObjectAabbMin, collisionObjectAabbMax);
                AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
                double[] hitLambda = new double[]{1.0}; // could use resultCallback.closestHitFraction, but needs testing
                Vector3d hitNormal = Stack.newVec();
                if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
                    CollisionWorld.objectQuerySingle(castShape, convexFromTrans, convexToTrans,
                            collisionObject,
                            collisionObject.getCollisionShape(),
                            collisionObject.getWorldTransform(tmpTrans),
                            resultCallback,
                            allowedCcdPenetration);
                }
            }
        }
    }

    @SuppressWarnings("unused")
    public void rayTest(Vector3d rayFromWorld, Vector3d rayToWorld, CollisionWorld.RayResultCallback resultCallback) {
        Transform rayFromTrans = Stack.newTrans();
        rayFromTrans.setIdentity();
        rayFromTrans.origin.set(rayFromWorld);
        Transform rayToTrans = Stack.newTrans();
        rayToTrans.setIdentity();
        rayToTrans.origin.set(rayToWorld);

        Transform tmpTrans = Stack.newTrans();

        for (int i = 0; i < overlappingObjects.getSize(); i++) {
            CollisionObject collisionObject = overlappingObjects.getQuick(i);

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.broadphaseHandle)) {
                CollisionWorld.rayTestSingle(rayFromTrans, rayToTrans,
                        collisionObject,
                        collisionObject.getCollisionShape(),
                        collisionObject.getWorldTransform(tmpTrans),
                        resultCallback);
            }
        }

        Stack.subTrans(3);
    }

    @SuppressWarnings("unused")
    public int getNumOverlappingObjects() {
        return overlappingObjects.getSize();
    }

    @SuppressWarnings("unused")
    public CollisionObject getOverlappingObject(int index) {
        return overlappingObjects.getQuick(index);
    }

    @SuppressWarnings("unused")
    public ObjectArrayList<CollisionObject> getOverlappingPairs() {
        return overlappingObjects;
    }

    public static GhostObject upcast(CollisionObject colObj) {
        if (colObj instanceof GhostObject) {
            return (GhostObject) colObj;
        }
        return null;
    }

}
