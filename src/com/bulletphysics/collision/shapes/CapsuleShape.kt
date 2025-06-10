package com.bulletphysics.collision.shapes

import com.bulletphysics.BulletGlobals
import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.MatrixUtil
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.VectorUtil.getCoord
import com.bulletphysics.linearmath.VectorUtil.mul
import com.bulletphysics.linearmath.VectorUtil.setCoord
import cz.advel.stack.Stack
import javax.vecmath.Vector3d
import kotlin.math.sqrt

/**
 * CapsuleShape represents a capsule around the Y axis, there is also the
 * [CapsuleShapeX] aligned around the X axis and [CapsuleShapeZ] around
 * the Z axis.
 *
 *
 *
 *
 * The total height is height+2*radius, so the height is just the height between
 * the center of each "sphere" of the capsule caps.
 *
 *
 *
 *
 * CapsuleShape is a convex hull of two spheres. The [MultiSphereShape] is
 * a more general collision shape that takes the convex hull of multiple sphere,
 * so it can also represent a capsule when just using two spheres.
 *
 * @author jezek2
 */
open class CapsuleShape : ConvexInternalShape {

    var upAxis: Int = 0

    // only used for CapsuleShapeZ and CapsuleShapeX subclasses.
    internal constructor()

    @JvmOverloads
    constructor(radius: Double, height: Double, axis: Int = 1) {
        upAxis = axis
        implicitShapeDimensions.set(radius, radius, radius)
        setCoord(implicitShapeDimensions, axis, 0.5 * height)
    }

    override fun localGetSupportingVertexWithoutMargin(dir: Vector3d, out: Vector3d): Vector3d {
        val supVec = out
        supVec.set(0.0, 0.0, 0.0)

        var maxDot = -1e308

        val vec = Stack.newVec(dir)
        val lenSqr = vec.lengthSquared()
        if (lenSqr < 0.0001f) {
            vec.set(1.0, 0.0, 0.0)
        } else {
            val rlen = 1.0 / sqrt(lenSqr)
            vec.scale(rlen)
        }

        val vtx = Stack.newVec()
        var newDot: Double

        val radius = this.radius

        val tmp1 = Stack.newVec()
        val tmp2 = Stack.newVec()
        val pos = Stack.newVec()

        run {
            pos.set(0.0, 0.0, 0.0)
            setCoord(pos, this.upAxis, this.halfHeight)

            mul(tmp1, vec, localScaling)
            tmp1.scale(radius)
            tmp2.scale(margin, vec)
            vtx.add(pos, tmp1)
            vtx.sub(tmp2)
            newDot = vec.dot(vtx)
            if (newDot > maxDot) {
                maxDot = newDot
                supVec.set(vtx)
            }
        }

        run {
            pos.set(0.0, 0.0, 0.0)
            setCoord(pos, this.upAxis, -this.halfHeight)

            mul(tmp1, vec, localScaling)
            tmp1.scale(radius)
            tmp2.scale(margin, vec)
            vtx.add(pos, tmp1)
            vtx.sub(tmp2)
            newDot = vec.dot(vtx)
            if (newDot > maxDot) {
                maxDot = newDot
                supVec.set(vtx)
            }
        }

        return out
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(
        dirs: Array<Vector3d>,
        outs: Array<Vector3d>,
        numVectors: Int
    ) {
        // TODO: implement
        throw UnsupportedOperationException("Not supported yet.")
    }

    override fun calculateLocalInertia(mass: Double, inertia: Vector3d) {
        // as an approximation, take the inertia of the box that bounds the spheres

        val ident = Stack.newTrans()
        ident.setIdentity()

        val radius = this.radius

        val halfExtents = Stack.newVec()
        halfExtents.set(radius, radius, radius)
        setCoord(halfExtents, this.upAxis, radius + this.halfHeight)

        val margin = BulletGlobals.CONVEX_DISTANCE_MARGIN

        val lx = 2.0 * (halfExtents.x + margin)
        val ly = 2.0 * (halfExtents.y + margin)
        val lz = 2.0 * (halfExtents.z + margin)
        val x2 = lx * lx
        val y2 = ly * ly
        val z2 = lz * lz
        val scaledmass = mass * 0.08333333f

        inertia.x = scaledmass * (y2 + z2)
        inertia.y = scaledmass * (x2 + z2)
        inertia.z = scaledmass * (x2 + y2)

        Stack.subVec(1)
        Stack.subTrans(1)
    }

    override val shapeType: BroadphaseNativeType
        get() = BroadphaseNativeType.CAPSULE_SHAPE_PROXYTYPE

    override fun getAabb(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        val tmp = Stack.newVec()

        val halfExtents = Stack.newVec()
        halfExtents.set(this.radius, this.radius, this.radius)
        setCoord(halfExtents, upAxis, this.radius + this.halfHeight)

        halfExtents.x += margin
        halfExtents.y += margin
        halfExtents.z += margin

        val absB = Stack.newMat()
        absB.set(t.basis)
        MatrixUtil.absolute(absB)

        val center = t.origin
        val extent = Stack.newVec()

        absB.getRow(0, tmp)
        extent.x = tmp.dot(halfExtents)
        absB.getRow(1, tmp)
        extent.y = tmp.dot(halfExtents)
        absB.getRow(2, tmp)
        extent.z = tmp.dot(halfExtents)

        aabbMin.sub(center, extent)
        aabbMax.add(center, extent)

        Stack.subVec(3)
        Stack.subMat(1)
    }

    val radius: Double
        get() {
            val radiusAxis = (upAxis + 2) % 3
            return getCoord(implicitShapeDimensions, radiusAxis)
        }

    val halfHeight: Double
        get() = getCoord(implicitShapeDimensions, upAxis)
}
