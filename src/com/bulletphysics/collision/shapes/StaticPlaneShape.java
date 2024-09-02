package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * StaticPlaneShape simulates an infinite non-moving (static) collision plane.
 * 
 * @author jezek2
 */
public class StaticPlaneShape extends ConcaveShape {

	protected final Vector3d localAabbMin = new Vector3d();
	protected final Vector3d localAabbMax = new Vector3d();
	
	protected final Vector3d planeNormal = new Vector3d();
	protected double planeConstant;
	protected final Vector3d localScaling = new Vector3d(0.0, 0.0, 0.0);

	public StaticPlaneShape(Vector3d planeNormal, double planeConstant) {
		this.planeNormal.normalize(planeNormal);
		this.planeConstant = planeConstant;
	}

	public Vector3d getPlaneNormal(Vector3d out) {
		out.set(planeNormal);
		return out;
	}

	public double getPlaneConstant() {
		return planeConstant;
	}
	
	@Override
	public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
		Vector3d tmp = Stack.newVec();
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();

		Vector3d halfExtents = Stack.newVec();
		halfExtents.sub(aabbMax, aabbMin);
		halfExtents.scale(0.5);

		double radius = halfExtents.length();
		Vector3d center = Stack.newVec();
		center.add(aabbMax, aabbMin);
		center.scale(0.5);

		// this is where the triangles are generated, given AABB and plane equation (normal/constant)

		Vector3d tangentDir0 = Stack.newVec(), tangentDir1 = Stack.newVec();

		// tangentDir0/tangentDir1 can be precalculated
		TransformUtil.planeSpace1(planeNormal, tangentDir0, tangentDir1);

		Vector3d supVertex0 = Stack.newVec(), supVertex1 = Stack.newVec();

		Vector3d projectedCenter = Stack.newVec();
		tmp.scale(planeNormal.dot(center) - planeConstant, planeNormal);
		projectedCenter.sub(center, tmp);

		Vector3d[] triangle = new Vector3d[] { Stack.newVec(), Stack.newVec(), Stack.newVec() };

		tmp1.scale(radius, tangentDir0);
		tmp2.scale(radius, tangentDir1);
		VectorUtil.add(triangle[0], projectedCenter, tmp1, tmp2);

		tmp1.scale(radius, tangentDir0);
		tmp2.scale(radius, tangentDir1);
		tmp.sub(tmp1, tmp2);
		VectorUtil.add(triangle[1], projectedCenter, tmp);

		tmp1.scale(radius, tangentDir0);
		tmp2.scale(radius, tangentDir1);
		tmp.sub(tmp1, tmp2);
		triangle[2].sub(projectedCenter, tmp);

		callback.processTriangle(triangle, 0, 0);

		tmp1.scale(radius, tangentDir0);
		tmp2.scale(radius, tangentDir1);
		tmp.sub(tmp1, tmp2);
		triangle[0].sub(projectedCenter, tmp);

		tmp1.scale(radius, tangentDir0);
		tmp2.scale(radius, tangentDir1);
		tmp.add(tmp1, tmp2);
		triangle[1].sub(projectedCenter, tmp);

		tmp1.scale(radius, tangentDir0);
		tmp2.scale(radius, tangentDir1);
		VectorUtil.add(triangle[2], projectedCenter, tmp1, tmp2);

		callback.processTriangle(triangle, 0, 1);
	}

	@Override
	public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		aabbMin.set(-1e30, -1e30, -1e30);
		aabbMax.set(1e30, 1e30, 1e30);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.STATIC_PLANE_PROXYTYPE;
	}

	@Override
	public void setLocalScaling(Vector3d scaling) {
		localScaling.set(scaling);
	}

	@Override
	public Vector3d getLocalScaling(Vector3d out) {
		out.set(localScaling);
		return out;
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		//moving concave objects not supported
		inertia.set(0.0, 0.0, 0.0);
	}

	@Override
	public String getName() {
		return "STATICPLANE";
	}

}
