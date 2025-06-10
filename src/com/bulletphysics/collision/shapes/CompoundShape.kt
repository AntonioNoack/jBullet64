package com.bulletphysics.collision.shapes

import com.bulletphysics.collision.broadphase.BroadphaseNativeType
import com.bulletphysics.linearmath.MatrixUtil
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.VectorUtil.getCoord
import com.bulletphysics.linearmath.VectorUtil.setCoord
import com.bulletphysics.linearmath.VectorUtil.setMax
import com.bulletphysics.linearmath.VectorUtil.setMin
import com.bulletphysics.util.ObjectArrayList
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

// JAVA NOTE: CompoundShape from 2.71
/**
 * CompoundShape allows to store multiple other [CollisionShape]s. This allows
 * for moving concave collision objects. This is more general than the [BvhTriangleMeshShape].
 *
 * @author jezek2
 */
class CompoundShape : CollisionShape() {
    val childList: ObjectArrayList<CompoundShapeChild?> = ObjectArrayList<CompoundShapeChild?>()
    private val localAabbMin = Vector3d(1e308, 1e308, 1e308)
    private val localAabbMax = Vector3d(-1e308, -1e308, -1e308)

    init {
        margin = 0.0
    }

    val localScaling: Vector3d = Vector3d(1.0, 1.0, 1.0)

    @Suppress("unused")
    fun addChildShape(localTransform: Transform, shape: CollisionShape) {
        val child = CompoundShapeChild()
        child.transform.set(localTransform)
        child.childShape = shape
        child.childShapeType = shape.shapeType
        child.childMargin = shape.margin

        childList.add(child)

        // extend the local aabbMin/aabbMax
        val localAabbMin = Stack.newVec()
        val localAabbMax = Stack.newVec()
        shape.getAabb(localTransform, localAabbMin, localAabbMax)

        setMin(this.localAabbMin, localAabbMin)
        setMax(this.localAabbMax, localAabbMax)
        Stack.subVec(2)
    }

    /**
     * Remove all children shapes that contain the specified shape.
     */
    @Suppress("unused")
    fun removeChildShape(shape: CollisionShape?) {
        var doneRemoving: Boolean

        // Find the children containing the shape specified, and remove those children.
        do {
            doneRemoving = true
            for (i in childList.indices) {
                if (childList.getQuick(i)!!.childShape === shape) {
                    childList.removeQuick(i)
                    doneRemoving = false // Do another iteration pass after removing from the vector
                    break
                }
            }
        } while (!doneRemoving)

        recalculateLocalAabb()
    }

    val numChildShapes: Int
        get() = childList.size

    fun getChildShape(index: Int): CollisionShape? {
        return childList.getQuick(index)!!.childShape
    }

    fun getChildTransform(index: Int, out: Transform): Transform {
        out.set(childList.getQuick(index)!!.transform)
        return out
    }

    /**
     * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
     */
    override fun getAabb(t: Transform, aabbMin: Vector3d, aabbMax: Vector3d) {
        val localHalfExtents = Stack.newVec()
        localHalfExtents.sub(localAabbMax, localAabbMin)
        localHalfExtents.scale(0.5)
        localHalfExtents.x += margin
        localHalfExtents.y += margin
        localHalfExtents.z += margin

        val localCenter = Stack.newVec()
        localCenter.add(localAabbMax, localAabbMin)
        localCenter.scale(0.5)

        val abs_b = Stack.newMat(t.basis)
        MatrixUtil.absolute(abs_b)

        val center = Stack.newVec(localCenter)
        t.transform(center)

        val tmp = Stack.newVec()

        val extent = Stack.newVec()
        abs_b.getRow(0, tmp)
        extent.x = tmp.dot(localHalfExtents)
        abs_b.getRow(1, tmp)
        extent.y = tmp.dot(localHalfExtents)
        abs_b.getRow(2, tmp)
        extent.z = tmp.dot(localHalfExtents)

        aabbMin.sub(center, extent)
        aabbMax.add(center, extent)
    }

    /**
     * Re-calculate the local Aabb. Is called at the end of removeChildShapes.
     * Use this yourself if you modify the children or their transforms.
     */
    fun recalculateLocalAabb() {
        // Recalculate the local aabb
        // Brute force, it iterates over all the shapes left.
        localAabbMin.set(1e308, 1e308, 1e308)
        localAabbMax.set(-1e308, -1e308, -1e308)

        val tmpLocalAabbMin = Stack.newVec()
        val tmpLocalAabbMax = Stack.newVec()

        // extend the local aabbMin/aabbMax
        for (j in childList.indices) {
            childList.getQuick(j)!!.childShape!!.getAabb(
                childList.getQuick(j)!!.transform,
                tmpLocalAabbMin,
                tmpLocalAabbMax
            )

            for (i in 0..2) {
                if (getCoord(localAabbMin, i) > getCoord(tmpLocalAabbMin, i)) {
                    setCoord(localAabbMin, i, getCoord(tmpLocalAabbMin, i))
                }
                if (getCoord(localAabbMax, i) < getCoord(tmpLocalAabbMax, i)) {
                    setCoord(localAabbMax, i, getCoord(tmpLocalAabbMax, i))
                }
            }
        }
    }

    override fun setLocalScaling(scaling: Vector3d) {
        localScaling.set(scaling)
    }

    override fun getLocalScaling(out: Vector3d): Vector3d {
        out.set(localScaling)
        return out
    }

    override fun calculateLocalInertia(mass: Double, inertia: Vector3d) {
        // approximation: take the inertia from the aabb for now
        val ident = Stack.newTrans()
        ident.setIdentity()
        val aabbMin = Stack.newVec()
        val aabbMax = Stack.newVec()
        getAabb(ident, aabbMin, aabbMax)

        val halfExtents = Stack.newVec()
        halfExtents.sub(aabbMax, aabbMin)
        halfExtents.scale(0.5)

        val lx = 2.0 * halfExtents.x
        val ly = 2.0 * halfExtents.y
        val lz = 2.0 * halfExtents.z

        inertia.x = (mass / 12f) * (ly * ly + lz * lz)
        inertia.y = (mass / 12f) * (lx * lx + lz * lz)
        inertia.z = (mass / 12f) * (lx * lx + ly * ly)

        Stack.subVec(3)
        Stack.subTrans(1)
    }

    override val shapeType: BroadphaseNativeType
        get() = BroadphaseNativeType.COMPOUND_SHAPE_PROXYTYPE

    /**
     * Computes the exact moment of inertia and the transform from the coordinate
     * system defined by the principal axes of the moment of inertia and the center
     * of mass to the current coordinate system. "masses" points to an array
     * of masses of the children. The resulting transform "principal" has to be
     * applied inversely to all children transforms in order for the local coordinate
     * system of the compound shape to be centered at the center of mass and to coincide
     * with the principal axes. This also necessitates a correction of the world transform
     * of the collision object by the principal transform.
     */
    @Suppress("unused")
    fun calculatePrincipalAxisTransform(masses: DoubleArray, principal: Transform, inertia: Vector3d) {
        val n = childList.size

        var totalMass = 0.0
        val center = Stack.newVec()
        center.set(0.0, 0.0, 0.0)
        for (k in 0..<n) {
            center.scaleAdd(masses[k], childList.getQuick(k)!!.transform.origin, center)
            totalMass += masses[k]
        }
        center.scale(1.0 / totalMass)
        principal.origin.set(center)

        val tensor = Stack.newMat()
        tensor.setZero()

        for (k in 0..<n) {
            val i = Stack.newVec()
            childList.getQuick(k)!!.childShape!!.calculateLocalInertia(masses[k], i)

            val t = childList.getQuick(k)!!.transform
            val o = Stack.newVec()
            o.sub(t.origin, center)

            // compute inertia tensor in coordinate system of compound shape
            val j = Stack.newMat()
            j.transpose(t.basis)

            j.m00 *= i.x
            j.m01 *= i.x
            j.m02 *= i.x
            j.m10 *= i.y
            j.m11 *= i.y
            j.m12 *= i.y
            j.m20 *= i.z
            j.m21 *= i.z
            j.m22 *= i.z

            j.mul(t.basis, j)

            // add inertia tensor
            tensor.add(j)

            // compute inertia tensor of pointmass at o
            val o2 = o.lengthSquared()
            j.setRow(0, o2, 0.0, 0.0)
            j.setRow(1, 0.0, o2, 0.0)
            j.setRow(2, 0.0, 0.0, o2)
            j.m00 += o.x * -o.x
            j.m01 += o.y * -o.x
            j.m02 += o.z * -o.x
            j.m10 += o.x * -o.y
            j.m11 += o.y * -o.y
            j.m12 += o.z * -o.y
            j.m20 += o.x * -o.z
            j.m21 += o.y * -o.z
            j.m22 += o.z * -o.z

            // add inertia tensor of pointmass
            tensor.m00 += masses[k] * j.m00
            tensor.m01 += masses[k] * j.m01
            tensor.m02 += masses[k] * j.m02
            tensor.m10 += masses[k] * j.m10
            tensor.m11 += masses[k] * j.m11
            tensor.m12 += masses[k] * j.m12
            tensor.m20 += masses[k] * j.m20
            tensor.m21 += masses[k] * j.m21
            tensor.m22 += masses[k] * j.m22
        }

        MatrixUtil.diagonalize(tensor, principal.basis, 0.00001, 20)

        inertia.set(tensor.m00, tensor.m11, tensor.m22)
    }
}
