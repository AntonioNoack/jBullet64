package com.bulletphysics.collision.broadphase

/**
 * Dbvt implementation by Nathanael Presson
 * @author jezek2
 */
class DbvtProxy(userPtr: Any?, collisionFilterGroup: Short, collisionFilterMask: Short) :
    BroadphaseProxy(userPtr, collisionFilterGroup, collisionFilterMask) {
    val aabb: DbvtAabbMm = DbvtAabbMm()
    var leaf: DbvtNode? = null
    val links: Array<DbvtProxy?> = arrayOfNulls<DbvtProxy>(2)
    var stage: Int = 0
}
