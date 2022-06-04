package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 *
 * @author jezek2
 */
class PlaneShape {

	public static void getPlaneEquation(StaticPlaneShape shape, Vector4d equation) {
		Vector3d tmp = Stack.newVec();
		equation.set(shape.getPlaneNormal(tmp));
		equation.w = shape.getPlaneConstant();
	}
	
	public static void getPlaneEquationTransformed(StaticPlaneShape shape, Transform trans, Vector4d equation) {
		getPlaneEquation(shape, equation);

		Vector3d tmp = Stack.newVec();

		trans.basis.getRow(0, tmp);
		double x = VectorUtil.dot3(tmp, equation);
		trans.basis.getRow(1, tmp);
		double y = VectorUtil.dot3(tmp, equation);
		trans.basis.getRow(2, tmp);
		double z = VectorUtil.dot3(tmp, equation);

		double w = VectorUtil.dot3(trans.origin, equation) + equation.w;

		equation.set(x, y, z, w);
	}
	
}
