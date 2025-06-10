package com.bulletphysics.collision.shapes

import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.Transform
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * UniformScalingShape allows to re-use uniform scaled instances of [ConvexShape]
 * in a memory efficient way. Istead of using [UniformScalingShape], it is better
 * to use the non-uniform setLocalScaling method on convex shapes that implement it.
 *
 * @author jezek2
 */
class UniformScalingShape(val childShape: ConvexShape, val uniformScalingFactor: Double) : ConvexShape() {
    override fun localGetSupportingVertex(dir: Vector3d, out: Vector3d): Vector3d {
        childShape.localGetSupportingVertex(dir, out)
        out.scale(uniformScalingFactor)
        return out
    }

    override fun localGetSupportingVertexWithoutMargin(dir: Vector3d, out: Vector3d): Vector3d {
        childShape.localGetSupportingVertexWithoutMargin(dir, out)
        out.scale(uniformScalingFactor)
        return out
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(
        dirs: Array<Vector3d>, outs: Array<Vector3d>, numVectors: Int
    ) {
        childShape.batchedUnitVectorGetSupportingVertexWithoutMargin(dirs, outs, numVectors)
        for (i in 0..<numVectors) {
            outs[i].scale(uniformScalingFactor)
        }
    }

    override fun getAabbSlow(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        childShape.getAabbSlow(t, aabbMin, aabbMax)
        val aabbCenter = Stack.newVec()
        aabbCenter.add(aabbMax, aabbMin)
        aabbCenter.scale(0.5)

        val scaledAabbHalfExtends = Stack.newVec()
        scaledAabbHalfExtends.sub(aabbMax, aabbMin)
        scaledAabbHalfExtends.scale(0.5 * uniformScalingFactor)

        aabbMin.sub(aabbCenter, scaledAabbHalfExtends)
        aabbMax.add(aabbCenter, scaledAabbHalfExtends)
    }

    override fun setLocalScaling(scaling: Vector3d) {
        childShape.setLocalScaling(scaling)
    }

    override fun getLocalScaling(out: Vector3d): Vector3d {
        childShape.getLocalScaling(out)
        return out
    }

    override var margin: Double
        get() = childShape.margin * uniformScalingFactor
        set(margin) {
            childShape.margin = margin
        }

    override val numPreferredPenetrationDirections: Int
        get() = childShape.numPreferredPenetrationDirections

    override fun getPreferredPenetrationDirection(index: Int, penetrationVector: Vector3d) {
        childShape.getPreferredPenetrationDirection(index, penetrationVector)
    }

    override fun getAabb(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        childShape.getAabb(t, aabbMin, aabbMax)
        val aabbCenter = Stack.newVec()
        aabbCenter.add(aabbMax, aabbMin)
        aabbCenter.scale(0.5)

        val scaledAabbHalfExtends = Stack.newVec()
        scaledAabbHalfExtends.sub(aabbMax, aabbMin)
        scaledAabbHalfExtends.scale(0.5 * uniformScalingFactor)

        aabbMin.sub(aabbCenter, scaledAabbHalfExtends)
        aabbMax.add(aabbCenter, scaledAabbHalfExtends)
        Stack.subVec(2)
    }

    override val shapeType: BroadphaseNativeType
        get() = BroadphaseNativeType.UNIFORM_SCALING_SHAPE_PROXYTYPE

    override fun calculateLocalInertia(mass: Double, inertia: Vector3d) {
        // this linear upscaling is not realistic, but we don't deal with large mass ratios...
        childShape.calculateLocalInertia(mass, inertia)
        inertia.scale(uniformScalingFactor)
    }
}
