package com.bulletphysics.extras.gimpact

import javax.vecmath.Matrix3d
import javax.vecmath.Vector3d
import kotlin.math.abs

/**
 * @author jezek2
 */
internal object BoxCollision {

    const val BOX_PLANE_EPSILON: Double = 0.000001

    fun absGreater(i: Double, j: Double): Boolean {
        return abs(i) > j
    }

    fun max(a: Double, b: Double, c: Double): Double {
        return kotlin.math.max(a, kotlin.math.max(b, c))
    }

    fun min(a: Double, b: Double, c: Double): Double {
        return kotlin.math.min(a, kotlin.math.min(b, c))
    }

    /**
     * Returns the dot product between a vec3 and the col of a matrix.
     */
    fun matXVec(mat: Matrix3d, vec3: Vector3d, column: Int): Double {
        return vec3.x * mat.getElement(0, column) + vec3.y * mat.getElement(1, column) + vec3.z * mat.getElement(
            2,
            column
        )
    }

}
