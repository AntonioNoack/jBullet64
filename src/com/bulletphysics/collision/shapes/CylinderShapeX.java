package com.bulletphysics.collision.shapes;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * Cylinder shape around the X axis.
 *
 * @author jezek2
 */
public class CylinderShapeX extends CylinderShape {

    public CylinderShapeX(Vector3d halfExtents) {
        super(halfExtents, false);
        upAxis = 0;
        recalculateLocalAabb();
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        return cylinderLocalSupportX(getHalfExtentsWithoutMargin(Stack.newVec()), dir, out);
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        for (int i = 0; i < numVectors; i++) {
            cylinderLocalSupportX(getHalfExtentsWithoutMargin(Stack.newVec()), vectors[i], supportVerticesOut[i]);
        }
    }

    @Override
    public double getRadius() {
        double r = getHalfExtentsWithMargin(Stack.newVec()).y;
        Stack.subVec(1);
        return r;
    }

    @Override
    public String getName() {
        return "CylinderX";
    }

}
