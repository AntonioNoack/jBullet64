package com.bulletphysics.extras.gimpact

import cz.advel.stack.Stack
import javax.vecmath.Vector3d
import javax.vecmath.Vector4d

/**
 * @author jezek2
 */
internal object GeometryOperations {
    /**
     * Calc a plane from a triangle edge and a normal.
     */
    @JvmStatic
    fun edgePlane(e1: Vector3d, e2: Vector3d, normal: Vector3d, plane: Vector4d) {
        val planeNormal = Stack.newVec()
        planeNormal.sub(e2, e1)
        planeNormal.cross(planeNormal, normal)
        planeNormal.normalize()

        plane.set(planeNormal)
        plane.w = e2.dot(planeNormal)
    }
}
