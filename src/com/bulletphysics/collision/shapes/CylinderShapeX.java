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
        super(halfExtents);
        upAxis = 0;
        recalculateLocalAabb();
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        Vector3d halfExtends = getHalfExtentsWithoutMargin(Stack.newVec());
        Vector3d result = cylinderLocalSupportX(halfExtends, dir, out);
        Stack.subVec(1);
        return result;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        Vector3d halfExtends = getHalfExtentsWithoutMargin(Stack.newVec());
        for (int i = 0; i < numVectors; i++) {
            cylinderLocalSupportX(halfExtends, vectors[i], supportVerticesOut[i]);
        }
        Stack.subVec(1);
    }

    @Override
    public double getRadius() {
        Vector3d tmp = Stack.newVec();
        double r = getHalfExtentsWithMargin(tmp).y;
        Stack.subVec(1);
        return r;
    }
}
