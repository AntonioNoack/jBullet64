package com.bulletphysics.collision.shapes;

import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * Cylinder shape around the Z axis.
 * 
 * @author jezek2
 */
public class CylinderShapeZ extends CylinderShape {

	public CylinderShapeZ(Vector3d halfExtents) {
		super(halfExtents, false);
		upAxis = 2;
		recalculateLocalAabb();
	}

	@Override
	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
		return cylinderLocalSupportZ(getHalfExtentsWithoutMargin(Stack.newVec()), dir, out);
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
		for (int i = 0; i < numVectors; i++) {
			cylinderLocalSupportZ(getHalfExtentsWithoutMargin(Stack.newVec()), vectors[i], supportVerticesOut[i]);
		}
	}

	@Override
	public double getRadius() {
		double r = getHalfExtentsWithMargin(Stack.newVec()).x;
		Stack.subVec(1);
		return r;
	}

	@Override
	public String getName() {
		return "CylinderZ";
	}
	
}
