package com.bulletphysics.collision.shapes

import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.Transform
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * SphereShape implements an implicit sphere, centered around a local origin with radius.
 *
 * @author jezek2
 */
class SphereShape(radius: Double) : ConvexInternalShape() {

    init {
        margin = radius
    }

    val radius get() = margin

    override fun localGetSupportingVertexWithoutMargin(vec: Vector3d, out: Vector3d): Vector3d {
        out.set(0.0, 0.0, 0.0)
        return out
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(
        vectors: Array<Vector3d>, supportVerticesOut: Array<Vector3d>, numVectors: Int
    ) {
        for (i in 0 until numVectors) {
            supportVerticesOut[i].set(0.0, 0.0, 0.0)
        }
    }

    override fun getAabb(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        val center = t.origin
        val extent = Stack.borrowVec()
        val margin = this.margin
        extent.set(margin, margin, margin)
        aabbMin.sub(center, extent)
        aabbMax.add(center, extent)
    }

    override val shapeType: BroadphaseNativeType
        get() = BroadphaseNativeType.SPHERE_SHAPE_PROXYTYPE

    override fun calculateLocalInertia(mass: Double, inertia: Vector3d) {
        val radius = this.margin
        val elem = 0.4 * mass * radius * radius
        inertia.set(elem, elem, elem)
    }
}
