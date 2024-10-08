package com.bulletphysics.extras.gimpact;

import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
class Quantization {

	public static void calcQuantizationParameters(Vector3d outMinBound, Vector3d outMaxBound, Vector3d bvhQuantization, Vector3d srcMinBound, Vector3d srcMaxBound, double quantizationMargin) {
		// enlarge the AABB to avoid division by zero when initializing the quantization values
		Vector3d clampValue = Stack.newVec();
		clampValue.set(quantizationMargin, quantizationMargin, quantizationMargin);
		outMinBound.sub(srcMinBound, clampValue);
		outMaxBound.add(srcMaxBound, clampValue);
		Vector3d aabbSize = Stack.newVec();
		aabbSize.sub(outMaxBound, outMinBound);
		bvhQuantization.set(65535.0, 65535.0, 65535.0);
		VectorUtil.div(bvhQuantization, bvhQuantization, aabbSize);
	}

	public static void quantizeClamp(short[] out, Vector3d point, Vector3d min_bound, Vector3d max_bound, Vector3d bvhQuantization) {
		Vector3d clampedPoint = Stack.newVec(point);
		VectorUtil.setMax(clampedPoint, min_bound);
		VectorUtil.setMin(clampedPoint, max_bound);

		Vector3d v = Stack.newVec();
		v.sub(clampedPoint, min_bound);
		VectorUtil.mul(v, v, bvhQuantization);

		out[0] = (short) (v.x + 0.5);
		out[1] = (short) (v.y + 0.5);
		out[2] = (short) (v.z + 0.5);
		Stack.subVec(2);
	}

	public static Vector3d dequantize(short[] vecIn, Vector3d offset, Vector3d bvhQuantization, Vector3d out) {
		out.set((double)(vecIn[0] & 0xFFFF) / (bvhQuantization.x),
		        (double)(vecIn[1] & 0xFFFF) / (bvhQuantization.y),
		        (double)(vecIn[2] & 0xFFFF) / (bvhQuantization.z));
		out.add(offset);
		return out;
	}
	
}
