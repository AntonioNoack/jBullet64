package com.bulletphysics.collision.shapes

import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.MatrixUtil.absolute
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.VectorUtil.mul
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

// JAVA NOTE: ScaledBvhTriangleMeshShape from 2.73 SP1
/**
 * The ScaledBvhTriangleMeshShape allows to instance a scaled version of an existing
 * [BvhTriangleMeshShape]. Note that each [BvhTriangleMeshShape] still can
 * have its own local scaling, independent of ScaledBvhTriangleMeshShape 'localScaling'.
 *
 * @author jezek2
 */
class ScaledBvhTriangleMeshShape @Suppress("unused") constructor(
    var childShape: BvhTriangleMeshShape,
    localScaling: Vector3d
) : ConcaveShape() {

    val localScaling: Vector3d = Vector3d()

    init {
        this.localScaling.set(localScaling)
    }

    override fun processAllTriangles(callback: TriangleCallback, aabbMin: Vector3d, aabbMax: Vector3d) {
        val scaledCallback = ScaledTriangleCallback(callback, localScaling)

        val invLocalScaling = Stack.newVec()
        invLocalScaling.set(1.0 / localScaling.x, 1.0 / localScaling.y, 1.0 / localScaling.z)

        val scaledAabbMin = Stack.newVec()
        val scaledAabbMax = Stack.newVec()

        // support negative scaling
        scaledAabbMin.x = if (localScaling.x >= 0.0) aabbMin.x * invLocalScaling.x else aabbMax.x * invLocalScaling.x
        scaledAabbMin.y = if (localScaling.y >= 0.0) aabbMin.y * invLocalScaling.y else aabbMax.y * invLocalScaling.y
        scaledAabbMin.z = if (localScaling.z >= 0.0) aabbMin.z * invLocalScaling.z else aabbMax.z * invLocalScaling.z

        scaledAabbMax.x = if (localScaling.x <= 0.0) aabbMin.x * invLocalScaling.x else aabbMax.x * invLocalScaling.x
        scaledAabbMax.y = if (localScaling.y <= 0.0) aabbMin.y * invLocalScaling.y else aabbMax.y * invLocalScaling.y
        scaledAabbMax.z = if (localScaling.z <= 0.0) aabbMin.z * invLocalScaling.z else aabbMax.z * invLocalScaling.z

        childShape.processAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax)
    }

    override fun getAabb(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        val localAabbMin = childShape.getLocalAabbMin(Stack.newVec())
        val localAabbMax = childShape.getLocalAabbMax(Stack.newVec())

        val tmpLocalAabbMin = Stack.newVec()
        val tmpLocalAabbMax = Stack.newVec()
        mul(tmpLocalAabbMin, localAabbMin, localScaling)
        mul(tmpLocalAabbMax, localAabbMax, localScaling)

        localAabbMin.x = if (localScaling.x >= 0.0) tmpLocalAabbMin.x else tmpLocalAabbMax.x
        localAabbMin.y = if (localScaling.y >= 0.0) tmpLocalAabbMin.y else tmpLocalAabbMax.y
        localAabbMin.z = if (localScaling.z >= 0.0) tmpLocalAabbMin.z else tmpLocalAabbMax.z
        localAabbMax.x = if (localScaling.x <= 0.0) tmpLocalAabbMin.x else tmpLocalAabbMax.x
        localAabbMax.y = if (localScaling.y <= 0.0) tmpLocalAabbMin.y else tmpLocalAabbMax.y
        localAabbMax.z = if (localScaling.z <= 0.0) tmpLocalAabbMin.z else tmpLocalAabbMax.z

        val localHalfExtents = Stack.newVec()
        localHalfExtents.sub(localAabbMax, localAabbMin)
        localHalfExtents.scale(0.5)

        val margin = childShape.margin
        localHalfExtents.x += margin
        localHalfExtents.y += margin
        localHalfExtents.z += margin

        val localCenter = Stack.newVec()
        localCenter.add(localAabbMax, localAabbMin)
        localCenter.scale(0.5)

        val abs_b = Stack.newMat(t.basis)
        absolute(abs_b)

        val center = Stack.newVec(localCenter)
        t.transform(center)

        val extent = Stack.newVec()
        val tmp = Stack.newVec()
        abs_b.getRow(0, tmp)
        extent.x = tmp.dot(localHalfExtents)
        abs_b.getRow(1, tmp)
        extent.y = tmp.dot(localHalfExtents)
        abs_b.getRow(2, tmp)
        extent.z = tmp.dot(localHalfExtents)

        aabbMin.sub(center, extent)
        aabbMax.add(center, extent)
    }

    override val shapeType: BroadphaseNativeType
        get() = BroadphaseNativeType.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE

    override fun setLocalScaling(scaling: Vector3d) {
        localScaling.set(scaling)
    }

    override fun getLocalScaling(out: Vector3d): Vector3d {
        out.set(localScaling)
        return out
    }

    override fun calculateLocalInertia(mass: Double, inertia: Vector3d) {
    }

    /**///////////////////////////////////////////////////////////////////////// */
    private class ScaledTriangleCallback(
        private val originalCallback: TriangleCallback,
        private val localScaling: Vector3d
    ) : TriangleCallback {
        private val newTriangle = Array(3) { Vector3d() }

        override fun processTriangle(triangle: Array<Vector3d>, partId: Int, triangleIndex: Int) {
            mul(newTriangle[0], triangle[0], localScaling)
            mul(newTriangle[1], triangle[1], localScaling)
            mul(newTriangle[2], triangle[2], localScaling)
            originalCallback.processTriangle(newTriangle, partId, triangleIndex)
        }
    }
}
