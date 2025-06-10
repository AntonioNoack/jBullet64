package com.bulletphysics.collision.shapes

import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.Transform

/**
 * Compound shape child.
 *
 * @author jezek2
 */
class CompoundShapeChild {
    @JvmField
    val transform: Transform = Transform()

    @JvmField
    var childShape: CollisionShape? = null

    @JvmField
    var childShapeType: BroadphaseNativeType? = null

    @JvmField
    var childMargin: Double = 0.0

    override fun equals(other: Any?): Boolean {
        if (other !is CompoundShapeChild) return false
        val child = other
        return transform == child.transform && childShape === child.childShape && childShapeType == child.childShapeType && childMargin == child.childMargin
    }

    override fun hashCode(): Int {
        var hash = 7
        hash = 19 * hash + transform.hashCode()
        hash = 19 * hash + childShape.hashCode()
        hash = 19 * hash + childShapeType.hashCode()
        hash = 19 * hash + childMargin.hashCode()
        return hash
    }
}
