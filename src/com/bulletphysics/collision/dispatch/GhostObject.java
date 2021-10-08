/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

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

    protected ObjectArrayList<CollisionObject> overlappingObjects = new ObjectArrayList<CollisionObject>();

    public GhostObject() {
        this.internalType = CollisionObjectType.GHOST_OBJECT;
    }

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
            overlappingObjects.set(index, overlappingObjects.getQuick(overlappingObjects.size() - 1));
            overlappingObjects.removeQuick(overlappingObjects.size() - 1);
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
        Vector3d linVel = Stack.newVec();
        Vector3d angVel = Stack.newVec();
        TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1.0, linVel, angVel);
        Transform R = Stack.newTrans();
        R.setIdentity();
        R.setRotation(convexFromTrans.getRotation(Stack.newQuat()));
        castShape.calculateTemporalAabb(R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);

        Transform tmpTrans = Stack.newTrans();

        // go over all objects, and if the ray intersects their aabb + cast shape aabb,
        // do a ray-shape query using convexCaster (CCD)
        Vector3d collisionObjectAabbMin = Stack.newVec();
        Vector3d collisionObjectAabbMax = Stack.newVec();
        Vector3d hitNormal = Stack.newVec();
        double[] hitLambda = new double[1];
        for (int i = 0; i < overlappingObjects.size(); i++) {
            CollisionObject collisionObject = overlappingObjects.getQuick(i);

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
                //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
                collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(tmpTrans), collisionObjectAabbMin, collisionObjectAabbMax);
                AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
                hitLambda[0] = 1.0; // could use resultCallback.closestHitFraction, but needs testing
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

        Stack.subVec(7);
        Stack.subQuat(1);
        Stack.subTrans(4);

    }

    public void rayTest(Vector3d rayFromWorld, Vector3d rayToWorld, CollisionWorld.RayResultCallback resultCallback) {
        Transform rayFromTrans = Stack.newTrans();
        rayFromTrans.setIdentity();
        rayFromTrans.origin.set(rayFromWorld);
        Transform rayToTrans = Stack.newTrans();
        rayToTrans.setIdentity();
        rayToTrans.origin.set(rayToWorld);

        Transform tmpTrans = Stack.newTrans();

        for (int i = 0; i < overlappingObjects.size(); i++) {
            CollisionObject collisionObject = overlappingObjects.getQuick(i);

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
                CollisionWorld.rayTestSingle(rayFromTrans, rayToTrans,
                        collisionObject,
                        collisionObject.getCollisionShape(),
                        collisionObject.getWorldTransform(tmpTrans),
                        resultCallback);
            }
        }

        Stack.subTrans(3);

    }

    public int getNumOverlappingObjects() {
        return overlappingObjects.size();
    }

    public CollisionObject getOverlappingObject(int index) {
        return overlappingObjects.getQuick(index);
    }

    public ObjectArrayList<CollisionObject> getOverlappingPairs() {
        return overlappingObjects;
    }

    //
    // internal cast
    //

    public static GhostObject upcast(CollisionObject colObj) {
        if (colObj.getInternalType() == CollisionObjectType.GHOST_OBJECT) {
            return (GhostObject) colObj;
        }

        return null;
    }

}
