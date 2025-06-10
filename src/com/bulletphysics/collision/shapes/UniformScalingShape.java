package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * UniformScalingShape allows to re-use uniform scaled instances of {@link ConvexShape}
 * in a memory efficient way. Istead of using {@link UniformScalingShape}, it is better
 * to use the non-uniform setLocalScaling method on convex shapes that implement it.
 * 
 * @author jezek2
 */
public class UniformScalingShape extends ConvexShape {

	private final ConvexShape childConvexShape;
	private final double uniformScalingFactor;

	public UniformScalingShape(ConvexShape convexChildShape, double uniformScalingFactor) {
		this.childConvexShape = convexChildShape;
		this.uniformScalingFactor = uniformScalingFactor;
	}

	public double getUniformScalingFactor() {
		return uniformScalingFactor;
	}

	public ConvexShape getChildShape() {
		return childConvexShape;
	}
	
	@Override
	public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out) {
		childConvexShape.localGetSupportingVertex(dir, out);
		out.scale(uniformScalingFactor);
		return out;
	}

	@Override
	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
		childConvexShape.localGetSupportingVertexWithoutMargin(dir, out);
		out.scale(uniformScalingFactor);
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
		childConvexShape.batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
		for (int i=0; i<numVectors; i++) {
			supportVerticesOut[i].scale(uniformScalingFactor);
		}
	}

	@Override
	public void getAabbSlow(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		childConvexShape.getAabbSlow(t, aabbMin, aabbMax);
		Vector3d aabbCenter = Stack.newVec();
		aabbCenter.add(aabbMax, aabbMin);
		aabbCenter.scale(0.5);

		Vector3d scaledAabbHalfExtends = Stack.newVec();
		scaledAabbHalfExtends.sub(aabbMax, aabbMin);
		scaledAabbHalfExtends.scale(0.5 * uniformScalingFactor);

		aabbMin.sub(aabbCenter, scaledAabbHalfExtends);
		aabbMax.add(aabbCenter, scaledAabbHalfExtends);
	}

	@Override
	public void setLocalScaling(Vector3d scaling) {
		childConvexShape.setLocalScaling(scaling);
	}

	@Override
	public Vector3d getLocalScaling(Vector3d out) {
		childConvexShape.getLocalScaling(out);
		return out;
	}

	@Override
	public void setMargin(double margin) {
		childConvexShape.setMargin(margin);
	}

	@Override
	public double getMargin() {
		return childConvexShape.getMargin() * uniformScalingFactor;
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return childConvexShape.getNumPreferredPenetrationDirections();
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vector3d penetrationVector) {
		childConvexShape.getPreferredPenetrationDirection(index, penetrationVector);
	}

	@Override
	public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		childConvexShape.getAabb(t, aabbMin, aabbMax);
		Vector3d aabbCenter = Stack.newVec();
		aabbCenter.add(aabbMax, aabbMin);
		aabbCenter.scale(0.5);

		Vector3d scaledAabbHalfExtends = Stack.newVec();
		scaledAabbHalfExtends.sub(aabbMax, aabbMin);
		scaledAabbHalfExtends.scale(0.5 * uniformScalingFactor);

		aabbMin.sub(aabbCenter, scaledAabbHalfExtends);
		aabbMax.add(aabbCenter, scaledAabbHalfExtends);
		Stack.subVec(2);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.UNIFORM_SCALING_SHAPE_PROXYTYPE;
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		// this linear upscaling is not realistic, but we don't deal with large mass ratios...
		childConvexShape.calculateLocalInertia(mass, inertia);
		inertia.scale(uniformScalingFactor);
	}

}
