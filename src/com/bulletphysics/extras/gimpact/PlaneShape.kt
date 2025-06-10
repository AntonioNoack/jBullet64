package com.bulletphysics.extras.gimpact

import com.bulletphysics.collision.shapes.StaticPlaneShape
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.VectorUtil
import cz.advel.stack.Stack
import javax.vecmath.Vector4d

/**
 * @author jezek2
 */
internal object PlaneShape {
    fun getPlaneEquation(shape: StaticPlaneShape, equation: Vector4d) {
        val tmp = Stack.newVec()
        equation.set(shape.getPlaneNormal(tmp))
        equation.w = shape.planeConstant
    }

    @JvmStatic
	fun getPlaneEquationTransformed(shape: StaticPlaneShape, trans: Transform, equation: Vector4d) {
        getPlaneEquation(shape, equation)

        val tmp = Stack.newVec()

        trans.basis.getRow(0, tmp)
        val x = VectorUtil.dot3(tmp, equation)
        trans.basis.getRow(1, tmp)
        val y = VectorUtil.dot3(tmp, equation)
        trans.basis.getRow(2, tmp)
        val z = VectorUtil.dot3(tmp, equation)

        val w = VectorUtil.dot3(trans.origin, equation) + equation.w

        equation.set(x, y, z, w)
    }
}
