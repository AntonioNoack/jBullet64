package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * ConeShape implements a cone shape primitive, centered around the origin and
 * aligned with the Y axis. The {@link ConeShapeX} is aligned around the X axis
 * and {@link ConeShapeZ} around the Z axis.
 * 
 * @author jezek2
 */
public class ConeShape extends ConvexInternalShape {

	private final double sinAngle;
	private final double radius;
	private final double height;
	private final int[] coneIndices = new int[3];

	public ConeShape(double radius, double height) {
		this.radius = radius;
		this.height = height;
		setConeUpIndex(1);
		sinAngle = (radius / Math.sqrt(this.radius * this.radius + this.height * this.height));
	}

	public double getRadius() {
		return radius;
	}

	public double getHeight() {
		return height;
	}

	private Vector3d coneLocalSupport(Vector3d v, Vector3d out) {
		double halfHeight = height * 0.5;

		if (VectorUtil.getCoord(v, coneIndices[1]) > v.length() * sinAngle) {
			VectorUtil.setCoord(out, coneIndices[0], 0.0);
			VectorUtil.setCoord(out, coneIndices[1], halfHeight);
			VectorUtil.setCoord(out, coneIndices[2], 0.0);
			return out;
		}
		else {
			double v0 = VectorUtil.getCoord(v, coneIndices[0]);
			double v2 = VectorUtil.getCoord(v, coneIndices[2]);
			double s = Math.sqrt(v0 * v0 + v2 * v2);
			if (s > BulletGlobals.FLT_EPSILON) {
				double d = radius / s;
				VectorUtil.setCoord(out, coneIndices[0], VectorUtil.getCoord(v, coneIndices[0]) * d);
				VectorUtil.setCoord(out, coneIndices[1], -halfHeight);
				VectorUtil.setCoord(out, coneIndices[2], VectorUtil.getCoord(v, coneIndices[2]) * d);
				return out;
			} else {
				VectorUtil.setCoord(out, coneIndices[0], 0.0);
				VectorUtil.setCoord(out, coneIndices[1], -halfHeight);
				VectorUtil.setCoord(out, coneIndices[2], 0.0);
				return out;
			}
		}
	}

	@Override
	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
		return coneLocalSupport(dir, out);
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
		for (int i=0; i<numVectors; i++) {
			Vector3d vec = vectors[i];
			coneLocalSupport(vec, supportVerticesOut[i]);
		}
	}

	@Override
	public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out) {
		Vector3d supVertex = coneLocalSupport(dir, out);
		if (getMargin() != 0.0) {
			Vector3d vecNorm = Stack.newVec(dir);
			if (vecNorm.lengthSquared() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
				vecNorm.set(-1, -1, -1);
			}
			vecNorm.normalize();
			supVertex.scaleAdd(getMargin(), vecNorm, supVertex);
			Stack.subVec(1);
		}
		return supVertex;
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.CONE_SHAPE_PROXYTYPE;
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		Transform identity = Stack.newTrans();
		identity.setIdentity();
		Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
		getAabb(identity, aabbMin, aabbMax);

		Vector3d halfExtents = Stack.newVec();
		halfExtents.sub(aabbMax, aabbMin);
		halfExtents.scale(0.5);

		double margin = getMargin();

		double lx = halfExtents.x + margin;
		double ly = halfExtents.y + margin;
		double lz = halfExtents.z + margin;
		double x2 = lx * lx;
		double y2 = ly * ly;
		double z2 = lz * lz;

		inertia.set(y2 + z2, x2 + z2, x2 + y2);
		inertia.scale(mass / 3.0);

	}

	@Override
	public String getName() {
		return "Cone";
	}

	// choose upAxis index
	public void setConeUpIndex(int upIndex) {
		switch (upIndex) {
			case 0:
				coneIndices[0] = 1;
				coneIndices[1] = 0;
				coneIndices[2] = 2;
				break;

			case 1:
				coneIndices[0] = 0;
				coneIndices[1] = 1;
				coneIndices[2] = 2;
				break;

			case 2:
				coneIndices[0] = 0;
				coneIndices[1] = 2;
				coneIndices[2] = 1;
				break;

			default:
				assert (false);
		}
	}

	public int getConeUpIndex() {
		return coneIndices[1];
	}
	
}
