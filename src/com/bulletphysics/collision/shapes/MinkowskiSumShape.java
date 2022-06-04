package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * MinkowskiSumShape is only for advanced users. This shape represents implicit
 * based minkowski sum of two convex implicit shapes.
 * 
 * @author jezek2
 */
public class MinkowskiSumShape extends ConvexInternalShape {

	private final Transform transA = new Transform();
	private final Transform transB = new Transform();
	private final ConvexShape shapeA;
	private final ConvexShape shapeB;

	public MinkowskiSumShape(ConvexShape shapeA, ConvexShape shapeB) {
		this.shapeA = shapeA;
		this.shapeB = shapeB;
		this.transA.setIdentity();
		this.transB.setIdentity();
	}
	
	@Override
	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
		Vector3d tmp = Stack.newVec();
		Vector3d supVertexA = Stack.newVec();
		Vector3d supVertexB = Stack.newVec();

		// btVector3 supVertexA = m_transA(m_shapeA->localGetSupportingVertexWithoutMargin(-vec*m_transA.getBasis()));
		tmp.negate(dir);
		MatrixUtil.transposeTransform(tmp, tmp, transA.basis);
		shapeA.localGetSupportingVertexWithoutMargin(tmp, supVertexA);
		transA.transform(supVertexA);

		// btVector3 supVertexB = m_transB(m_shapeB->localGetSupportingVertexWithoutMargin(vec*m_transB.getBasis()));
		MatrixUtil.transposeTransform(tmp, dir, transB.basis);
		shapeB.localGetSupportingVertexWithoutMargin(tmp, supVertexB);
		transB.transform(supVertexB);

		//return supVertexA - supVertexB;
		out.sub(supVertexA, supVertexB);
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
		//todo: could make recursive use of batching. probably this shape is not used frequently.
		for (int i = 0; i < numVectors; i++) {
			localGetSupportingVertexWithoutMargin(vectors[i], supportVerticesOut[i]);
		}
	}

	@Override
	public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		throw new UnsupportedOperationException("Not supported yet.");
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.MINKOWSKI_SUM_SHAPE_PROXYTYPE;
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		assert (false);
		inertia.set(0, 0, 0);
	}

	@Override
	public String getName() {
		return "MinkowskiSum";
	}
	
	@Override
	public double getMargin() {
		return shapeA.getMargin() + shapeB.getMargin();
	}

	public void setTransformA(Transform transA) {
		this.transA.set(transA);
	}

	public void setTransformB(Transform transB) {
		this.transB.set(transB);
	}

	public void getTransformA(Transform dest) {
		dest.set(transA);
	}

	public void getTransformB(Transform dest) {
		dest.set(transB);
	}

}
