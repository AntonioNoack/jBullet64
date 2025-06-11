package vecmath

import org.joml.Quaterniond
import org.joml.Vector3d

class Matrix3d {

    val content = org.joml.Matrix3d()

    var m00: Double
        get() = content.m00
        set(value) {
            content.m00 = value
        }

    var m10: Double
        get() = content.m10
        set(value) {
            content.m10 = value
        }

    var m20: Double
        get() = content.m20
        set(value) {
            content.m20 = value
        }

    var m01: Double
        get() = content.m01
        set(value) {
            content.m01 = value
        }

    var m11: Double
        get() = content.m11
        set(value) {
            content.m11 = value
        }

    var m21: Double
        get() = content.m21
        set(value) {
            content.m21 = value
        }

    var m02: Double
        get() = content.m02
        set(value) {
            content.m02 = value
        }

    var m12: Double
        get() = content.m12
        set(value) {
            content.m12 = value
        }

    var m22: Double
        get() = content.m22
        set(value) {
            content.m22 = value
        }

    fun set(v: Matrix3d) {
        content.set(v.content)
    }

    /**
     * Used in Rem's Engine
     * */
    fun set(q: Quaterniond) {
        content.set(q)
    }

    fun transform(src: Vector3d) {
        revTransform(src, src)
    }

    fun revTransform(src: Vector3d, dst: Vector3d) {
       content.transform(src, dst)
    }

    fun getColumn(i: Int, v: Vector3d) {
        content.getColumn(i, v)
    }

    fun getRow(i: Int, v: Vector3d) {
        content.getRow(i, v)
    }

    fun transpose() {
        content.transpose()
    }

    fun setTranspose(src: Matrix3d) {
        src.content.transpose(content)
    }

    fun zero() {
        content.zero()
    }

    fun identity() {
        content.identity()
    }

    fun setMul(a: Matrix3d, b: Matrix3d) {
        a.content.mul(b.content, this.content)
    }

    fun mul(a: Matrix3d) {
        this.content.mul(a.content)
    }

    fun add(a: Matrix3d) {
        content.add(a.content)
    }

    fun setElement(i: Int, j: Int, v: Double) {
        // first number is fast on both implementations;
        // javax has stuff flipped -> flip it
        content[j, i] = v
    }

    fun getElement(i: Int, j: Int): Double {
        // first number is fast on both implementations;
        // javax has stuff flipped -> flip it
        return content[j, i]
    }

    fun setRow(i: Int, x: Double, y: Double, z: Double) {
        content.setRow(i, x, y, z)
    }

    fun setRow(i: Int, v: Vector3d) {
        setRow(i, v.x, v.y, v.z)
    }
}