package com.bulletphysics.linearmath

import com.bulletphysics.BulletGlobals
import com.bulletphysics.linearmath.VectorUtil.getCoord
import com.bulletphysics.linearmath.VectorUtil.setCoord
import com.bulletphysics.util.ArrayPool
import cz.advel.stack.Stack
import javax.vecmath.Matrix3d
import javax.vecmath.Quat4d
import javax.vecmath.Vector3d
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Utility functions for matrices.
 *
 * @author jezek2
 */
object MatrixUtil {
    @JvmStatic
    fun scale(dst: Matrix3d, mat: Matrix3d, s: Vector3d) {
        dst.m00 = mat.m00 * s.x
        dst.m01 = mat.m01 * s.y
        dst.m02 = mat.m02 * s.z
        dst.m10 = mat.m10 * s.x
        dst.m11 = mat.m11 * s.y
        dst.m12 = mat.m12 * s.z
        dst.m20 = mat.m20 * s.x
        dst.m21 = mat.m21 * s.y
        dst.m22 = mat.m22 * s.z
    }

    @JvmStatic
    fun absolute(mat: Matrix3d) {
        mat.m00 = abs(mat.m00)
        mat.m01 = abs(mat.m01)
        mat.m02 = abs(mat.m02)
        mat.m10 = abs(mat.m10)
        mat.m11 = abs(mat.m11)
        mat.m12 = abs(mat.m12)
        mat.m20 = abs(mat.m20)
        mat.m21 = abs(mat.m21)
        mat.m22 = abs(mat.m22)
    }

    private fun tdotx(mat: Matrix3d, vec: Vector3d): Double {
        return mat.m00 * vec.x + mat.m10 * vec.y + mat.m20 * vec.z
    }

    private fun tdoty(mat: Matrix3d, vec: Vector3d): Double {
        return mat.m01 * vec.x + mat.m11 * vec.y + mat.m21 * vec.z
    }

    private fun tdotz(mat: Matrix3d, vec: Vector3d): Double {
        return mat.m02 * vec.x + mat.m12 * vec.y + mat.m22 * vec.z
    }

    @JvmStatic
    fun transposeTransform(dst: Vector3d, vec: Vector3d, mat: Matrix3d) {
        val x = tdotx(mat, vec)
        val y = tdoty(mat, vec)
        val z = tdotz(mat, vec)
        dst.x = x
        dst.y = y
        dst.z = z
    }

    @JvmStatic
    fun setRotation(dst: Matrix3d, q: Quat4d) {
        val d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        assert(d != 0.0)
        val s = 2f / d
        val xs = q.x * s
        val ys = q.y * s
        val zs = q.z * s
        val wx = q.w * xs
        val wy = q.w * ys
        val wz = q.w * zs
        val xx = q.x * xs
        val xy = q.x * ys
        val xz = q.x * zs
        val yy = q.y * ys
        val yz = q.y * zs
        val zz = q.z * zs
        dst.m00 = 1.0 - (yy + zz)
        dst.m01 = xy - wz
        dst.m02 = xz + wy
        dst.m10 = xy + wz
        dst.m11 = 1.0 - (xx + zz)
        dst.m12 = yz - wx
        dst.m20 = xz - wy
        dst.m21 = yz + wx
        dst.m22 = 1.0 - (xx + yy)
    }

    @JvmStatic
    fun getRotation(mat: Matrix3d, dst: Quat4d) {
        val trace = mat.m00 + mat.m11 + mat.m22
        if (trace > 0.0) {
            var s = sqrt(trace + 1.0)
            dst.w = (s * 0.5)
            s = 0.5 / s

            dst.x = ((mat.m21 - mat.m12) * s)
            dst.y = ((mat.m02 - mat.m20) * s)
            dst.z = ((mat.m10 - mat.m01) * s)
        } else {
            val floatArrays = ArrayPool.Companion.get<DoubleArray?>(Double::class.javaPrimitiveType!!)
            val temp = floatArrays.getFixed(4)
            val i = if (mat.m00 < mat.m11) (if (mat.m11 < mat.m22) 2 else 1) else (if (mat.m00 < mat.m22) 2 else 0)
            val j = (i + 1) % 3
            val k = (i + 2) % 3

            var s = sqrt(mat.getElement(i, i) - mat.getElement(j, j) - mat.getElement(k, k) + 1.0)
            temp!![i] = s * 0.5
            s = 0.5 / s

            temp[3] = (mat.getElement(k, j) - mat.getElement(j, k)) * s
            temp[j] = (mat.getElement(j, i) + mat.getElement(i, j)) * s
            temp[k] = (mat.getElement(k, i) + mat.getElement(i, k)) * s
            dst.set(temp[0], temp[1], temp[2], temp[3])

            floatArrays.release(temp)
        }
    }

    private fun cofac(mat: Matrix3d, r1: Int, c1: Int, r2: Int, c2: Int): Double {
        return mat.getElement(r1, c1) * mat.getElement(r2, c2) - mat.getElement(r1, c2) * mat.getElement(r2, c1)
    }

    @JvmStatic
    fun invert(mat: Matrix3d) {
        val coX = cofac(mat, 1, 1, 2, 2)
        val coY = cofac(mat, 1, 2, 2, 0)
        val coZ = cofac(mat, 1, 0, 2, 1)

        val det = mat.m00 * coX + mat.m01 * coY + mat.m02 * coZ
        assert(det != 0.0)

        val s = 1.0 / det
        val m00 = coX * s
        val m01 = cofac(mat, 0, 2, 2, 1) * s
        val m02 = cofac(mat, 0, 1, 1, 2) * s
        val m10 = coY * s
        val m11 = cofac(mat, 0, 0, 2, 2) * s
        val m12 = cofac(mat, 0, 2, 1, 0) * s
        val m20 = coZ * s
        val m21 = cofac(mat, 0, 1, 2, 0) * s
        val m22 = cofac(mat, 0, 0, 1, 1) * s

        mat.m00 = m00
        mat.m01 = m01
        mat.m02 = m02
        mat.m10 = m10
        mat.m11 = m11
        mat.m12 = m12
        mat.m20 = m20
        mat.m21 = m21
        mat.m22 = m22
    }

    /**
     * Diagonalizes this matrix by the Jacobi method. rot stores the rotation
     * from the coordinate system in which the matrix is diagonal to the original
     * coordinate system, i.e., old_this = rot * new_this * rot^T. The iteration
     * stops when all off-diagonal elements are less than the threshold multiplied
     * by the sum of the absolute values of the diagonal, or when maxSteps have
     * been executed. Note that this matrix is assumed to be symmetric.
     */
    // JAVA NOTE: diagonalize method from 2.71
    fun diagonalize(mat: Matrix3d, rot: Matrix3d, threshold: Double, maxSteps: Int) {
        val row = Stack.newVec()

        rot.setIdentity()
        var step = maxSteps
        while (step > 0) {
            // find off-diagonal element [p][q] with largest magnitude
            var p = 0
            var q = 1
            var r = 2
            var max = abs(mat.m01)
            var v = abs(mat.m02)
            if (v > max) {
                q = 2
                r = 1
                max = v
            }
            v = abs(mat.m12)
            if (v > max) {
                p = 1
                q = 2
                r = 0
                max = v
            }

            var t = threshold * (abs(mat.m00) + abs(mat.m11) + abs(mat.m22))
            if (max <= t) {
                if (max <= BulletGlobals.SIMD_EPSILON * t) {
                    return
                }
                step = 1
            }

            // compute Jacobi rotation J which leads to a zero for element [p][q]
            val mpq = mat.getElement(p, q)
            val theta = (mat.getElement(q, q) - mat.getElement(p, p)) / (2 * mpq)
            val theta2 = theta * theta
            val cos: Double
            if ((theta2 * theta2) < (10f / BulletGlobals.SIMD_EPSILON)) {
                t = if (theta >= 0.0)
                    1.0 / (theta + sqrt(1.0 + theta2))
                else
                    1.0 / (theta - sqrt(1.0 + theta2))
                cos = 1.0 / sqrt(1.0 + t * t)
            } else {
                // approximation for large theta-value, i.e., a nearly diagonal matrix
                t = 1 / (theta * (2 + 0.5 / theta2))
                cos = 1 - 0.5 * t * t
            }
            val sin = cos * t

            // apply rotation to matrix (this = J^T * this * J)
            mat.setElement(p, q, 0.0)
            mat.setElement(q, p, 0.0)
            mat.setElement(p, p, mat.getElement(p, p) - t * mpq)
            mat.setElement(q, q, mat.getElement(q, q) + t * mpq)
            var mrp = mat.getElement(r, p)
            var mrq = mat.getElement(r, q)
            mat.setElement(r, p, cos * mrp - sin * mrq)
            mat.setElement(p, r, cos * mrp - sin * mrq)
            mat.setElement(r, q, cos * mrq + sin * mrp)
            mat.setElement(q, r, cos * mrq + sin * mrp)

            // apply rotation to rot (rot = rot * J)
            for (i in 0..2) {
                rot.getRow(i, row)

                mrp = getCoord(row, p)
                mrq = getCoord(row, q)
                setCoord(row, p, cos * mrp - sin * mrq)
                setCoord(row, q, cos * mrq + sin * mrp)
                rot.setRow(i, row)
            }
            step--
        }
    }
}
