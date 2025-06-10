package com.bulletphysics.collision.dispatch

import com.bulletphysics.collision.broadphase.BroadphaseProxy
import com.bulletphysics.collision.broadphase.Dispatcher
import com.bulletphysics.collision.dispatch.CollisionWorld.ConvexResultCallback
import com.bulletphysics.collision.dispatch.CollisionWorld.RayResultCallback
import com.bulletphysics.collision.shapes.ConvexShape
import com.bulletphysics.linearmath.AabbUtil2
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.TransformUtil
import com.bulletphysics.util.ObjectArrayList
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * GhostObject can keep track of all objects that are overlapping. By default, this
 * overlap is based on the AABB. This is useful for creating a character controller,
 * collision sensors/triggers, explosions etc.
 *
 * @author tomrbryn
 */
open class GhostObject : CollisionObject() {
    
    val overlappingPairs: ObjectArrayList<CollisionObject> = ObjectArrayList<CollisionObject>()

    /**
     * This method is mainly for expert/internal use only.
     */
    open fun addOverlappingObjectInternal(otherProxy: BroadphaseProxy, thisProxy: BroadphaseProxy?) {
        val otherObject: CollisionObject? = checkNotNull(otherProxy.clientObject as CollisionObject?)
        // if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
        val index = overlappingPairs.indexOf(otherObject)
        if (index == -1) {
            // not found
            overlappingPairs.add(otherObject)
        }
    }

    /**
     * This method is mainly for expert/internal use only.
     */
    open fun removeOverlappingObjectInternal(
        otherProxy: BroadphaseProxy,
        dispatcher: Dispatcher,
        thisProxy: BroadphaseProxy?
    ) {
        val otherObject: CollisionObject? = checkNotNull(otherProxy.clientObject as CollisionObject?)
        val index = overlappingPairs.indexOf(otherObject)
        if (index != -1) {
            overlappingPairs[index] = overlappingPairs.getQuick(overlappingPairs.size - 1)
            overlappingPairs.removeQuick(overlappingPairs.size - 1)
        }
    }

    fun convexSweepTest(
        castShape: ConvexShape,
        convexFromWorld: Transform,
        convexToWorld: Transform,
        resultCallback: ConvexResultCallback,
        allowedCcdPenetration: Double
    ) {
        val convexFromTrans = Stack.newTrans()
        val convexToTrans = Stack.newTrans()

        convexFromTrans.set(convexFromWorld)
        convexToTrans.set(convexToWorld)

        val castShapeAabbMin = Stack.newVec()
        val castShapeAabbMax = Stack.newVec()

        // compute AABB that encompasses angular movement
        run {
            val linVel = Stack.newVec()
            val angVel = Stack.newVec()
            TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1.0, linVel, angVel)
            val R = Stack.newTrans()
            R.setIdentity()
            R.setRotation(convexFromTrans.getRotation(Stack.newQuat()))
            castShape.calculateTemporalAabb(R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax)
        }

        val tmpTrans = Stack.newTrans()

        // go over all objects, and if the ray intersects their aabb + cast shape aabb,
        // do a ray-shape query using convexCaster (CCD)
        for (i in 0..<overlappingPairs.getSize()) {
            val collisionObject = overlappingPairs.getQuick(i)

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.broadphaseHandle!!)) {
                //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
                val collisionObjectAabbMin = Stack.newVec()
                val collisionObjectAabbMax = Stack.newVec()
                collisionObject.collisionShape!!.getAabb(
                    collisionObject.getWorldTransform(tmpTrans),
                    collisionObjectAabbMin,
                    collisionObjectAabbMax
                )
                AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax)
                val hitLambda = doubleArrayOf(1.0) // could use resultCallback.closestHitFraction, but needs testing
                val hitNormal = Stack.newVec()
                if (AabbUtil2.rayAabb(
                        convexFromWorld.origin,
                        convexToWorld.origin,
                        collisionObjectAabbMin,
                        collisionObjectAabbMax,
                        hitLambda,
                        hitNormal
                    )
                ) {
                    CollisionWorld.objectQuerySingle(
                        castShape, convexFromTrans, convexToTrans,
                        collisionObject,
                        collisionObject.collisionShape!!,
                        collisionObject.getWorldTransform(tmpTrans),
                        resultCallback,
                        allowedCcdPenetration
                    )
                }
            }
        }
    }

    @Suppress("unused")
    fun rayTest(rayFromWorld: Vector3d, rayToWorld: Vector3d, resultCallback: RayResultCallback) {
        val rayFromTrans = Stack.newTrans()
        rayFromTrans.setIdentity()
        rayFromTrans.origin.set(rayFromWorld)
        val rayToTrans = Stack.newTrans()
        rayToTrans.setIdentity()
        rayToTrans.origin.set(rayToWorld)

        val tmpTrans = Stack.newTrans()

        for (i in 0..<overlappingPairs.getSize()) {
            val collisionObject = overlappingPairs.getQuick(i)

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.broadphaseHandle!!)) {
                CollisionWorld.rayTestSingle(
                    rayFromTrans, rayToTrans,
                    collisionObject,
                    collisionObject.collisionShape!!,
                    collisionObject.getWorldTransform(tmpTrans),
                    resultCallback
                )
            }
        }

        Stack.subTrans(3)
    }

    val numOverlappingObjects: Int
        get() = overlappingPairs.size

    @Suppress("unused")
    fun getOverlappingObject(index: Int): CollisionObject? {
        return overlappingPairs.getQuick(index)
    }

    companion object {
        fun upcast(colObj: CollisionObject?): GhostObject? {
            if (colObj is GhostObject) {
                return colObj
            }
            return null
        }
    }
}
