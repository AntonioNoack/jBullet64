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
        super(halfExtents);
        upAxis = 2;
        recalculateLocalAabb();
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        Vector3d halfExtends = getHalfExtentsWithoutMargin(Stack.newVec());
        Vector3d result = cylinderLocalSupportZ(halfExtends, dir, out);
        Stack.subVec(1);
        return result;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        Vector3d halfExtends = getHalfExtentsWithoutMargin(Stack.newVec());
        for (int i = 0; i < numVectors; i++) {
            cylinderLocalSupportZ(halfExtends, vectors[i], supportVerticesOut[i]);
        }
        Stack.subVec(1);
    }
}
