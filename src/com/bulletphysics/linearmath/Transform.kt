package com.bulletphysics.linearmath

import com.bulletphysics.linearmath.MatrixUtil.getRotation
import com.bulletphysics.linearmath.MatrixUtil.setRotation
import cz.advel.stack.Stack
import javax.vecmath.Matrix3d
import javax.vecmath.Matrix4d
import javax.vecmath.Quat4d
import javax.vecmath.Vector3d

/**
 * Transform represents translation and rotation (rigid transform). Scaling and
 * shearing is not supported.
 *
 * You can use local shape scaling or [com.bulletphysics.collision.shapes.UniformScalingShape] for static rescaling
 * of collision objects.
 *
 * @author jezek2
 */
class Transform {
    /**
     * Rotation matrix of this Transform.
     */
    @JvmField
    val basis: Matrix3d = Matrix3d()

    /**
     * Translation vector of this Transform.
     */
    @JvmField
    val origin: Vector3d = Vector3d()

    constructor()

    constructor(mat: Matrix3d) {
        basis.set(mat)
    }

    constructor(mat: Matrix4d) {
        set(mat)
    }

    constructor(tr: Transform) {
        set(tr)
    }

    fun set(tr: Transform) {
        basis.set(tr.basis)
        origin.set(tr.origin)
    }

    fun set(mat: Matrix3d) {
        basis.set(mat)
        origin.set(0.0, 0.0, 0.0)
    }

    fun set(mat: Matrix4d) {
        mat.getRotationScale(basis)
        origin.set(mat.m03, mat.m13, mat.m23)
    }

    fun transform(v: Vector3d) {
        basis.transform(v)
        v.add(origin)
    }

    fun setIdentity() {
        basis.setIdentity()
        origin.set(0.0, 0.0, 0.0)
    }

    fun inverse() {
        basis.transpose()
        origin.scale(-1.0)
        basis.transform(origin)
    }

    fun inverse(tr: Transform) {
        set(tr)
        inverse()
    }

    fun mul(tr: Transform) {
        val vec = Stack.borrowVec(tr.origin)
        transform(vec)
        basis.mul(tr.basis)
        origin.set(vec)
    }

    fun mul(tr1: Transform, tr2: Transform) {
        val vec = Stack.borrowVec(tr2.origin)
        tr1.transform(vec)
        basis.mul(tr1.basis, tr2.basis)
        origin.set(vec)
    }

    fun invXform(inVec: Vector3d, out: Vector3d) {
        out.sub(inVec, origin)
        val mat = Stack.borrowMat(basis)
        mat.transpose()
        mat.transform(out)
    }

    fun getRotation(out: Quat4d): Quat4d {
        getRotation(basis, out)
        return out
    }

    fun setRotation(q: Quat4d) {
        setRotation(basis, q)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Transform) return false
        val tr = other
        return basis.equals(tr.basis) && origin.equals(tr.origin)
    }

    override fun hashCode(): Int {
        var hash = 3
        hash = 41 * hash + basis.hashCode()
        hash = 41 * hash + origin.hashCode()
        return hash
    }
}
