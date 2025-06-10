package com.bulletphysics.collision.shapes

import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.TransformUtil
import com.bulletphysics.linearmath.VectorUtil
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * StaticPlaneShape simulates an infinite non-moving (static) collision plane.
 *
 * @author jezek2
 */
class StaticPlaneShape(planeNormal: Vector3d, var planeConstant: Double) : ConcaveShape() {

    val planeNormal = Vector3d()
    val localScaling = Vector3d(0.0, 0.0, 0.0)

    init {
        this.planeNormal.normalize(planeNormal)
    }

    fun getPlaneNormal(out: Vector3d): Vector3d {
        out.set(planeNormal)
        return out
    }

    override fun processAllTriangles(callback: TriangleCallback, aabbMin: Vector3d, aabbMax: Vector3d) {
        val tmp = Stack.newVec()
        val tmp1 = Stack.newVec()
        val tmp2 = Stack.newVec()

        val halfExtents = Stack.newVec()
        halfExtents.sub(aabbMax, aabbMin)
        halfExtents.scale(0.5)

        val radius = halfExtents.length()
        val center = Stack.newVec()
        center.add(aabbMax, aabbMin)
        center.scale(0.5)

        // this is where the triangles are generated, given AABB and plane equation (normal/constant)
        val tangentDir0 = Stack.newVec()
        val tangentDir1 = Stack.newVec()

        // tangentDir0/tangentDir1 can be precalculated
        TransformUtil.planeSpace1(planeNormal, tangentDir0, tangentDir1)

        val projectedCenter = Stack.newVec()
        tmp.scale(planeNormal.dot(center) - planeConstant, planeNormal)
        projectedCenter.sub(center, tmp)

        val triangle = arrayOf(Stack.newVec(), Stack.newVec(), Stack.newVec())

        tmp1.scale(radius, tangentDir0)
        tmp2.scale(radius, tangentDir1)
        VectorUtil.add(triangle[0], projectedCenter, tmp1, tmp2)

        tmp1.scale(radius, tangentDir0)
        tmp2.scale(radius, tangentDir1)
        tmp.sub(tmp1, tmp2)
        VectorUtil.add(triangle[1], projectedCenter, tmp)

        tmp1.scale(radius, tangentDir0)
        tmp2.scale(radius, tangentDir1)
        tmp.sub(tmp1, tmp2)
        triangle[2]!!.sub(projectedCenter, tmp)

        callback.processTriangle(triangle, 0, 0)

        tmp1.scale(radius, tangentDir0)
        tmp2.scale(radius, tangentDir1)
        tmp.sub(tmp1, tmp2)
        triangle[0]!!.sub(projectedCenter, tmp)

        tmp1.scale(radius, tangentDir0)
        tmp2.scale(radius, tangentDir1)
        tmp.add(tmp1, tmp2)
        triangle[1]!!.sub(projectedCenter, tmp)

        tmp1.scale(radius, tangentDir0)
        tmp2.scale(radius, tangentDir1)
        VectorUtil.add(triangle[2], projectedCenter, tmp1, tmp2)

        callback.processTriangle(triangle, 0, 1)

        Stack.subVec(11)
    }

    override fun getAabb(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        aabbMin.set(-1e308, -1e308, -1e308)
        aabbMax.set(1e308, 1e308, 1e308)
    }

    override val shapeType: BroadphaseNativeType
        get() = BroadphaseNativeType.STATIC_PLANE_PROXYTYPE

    override fun setLocalScaling(scaling: Vector3d) {
        localScaling.set(scaling)
    }

    override fun getLocalScaling(out: Vector3d): Vector3d {
        out.set(localScaling)
        return out
    }

    override fun calculateLocalInertia(mass: Double, inertia: Vector3d) {
        //moving concave objects not supported
        inertia.set(0.0, 0.0, 0.0)
    }
}
