package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.BU_Simplex1to4;

import javax.vecmath.Vector3d;

/**
 * Helper class for tetrahedrons.
 *
 * @author jezek2
 */
class TetrahedronShapeEx extends BU_Simplex1to4 {

    public TetrahedronShapeEx() {
        numVertices = 4;
        for (int i = 0; i < numVertices; i++) {
            vertices[i] = new Vector3d();
        }
    }

    @SuppressWarnings("unused")
    public void setVertices(Vector3d v0, Vector3d v1, Vector3d v2, Vector3d v3) {
        vertices[0].set(v0);
        vertices[1].set(v1);
        vertices[2].set(v2);
        vertices[3].set(v3);
        recalculateLocalAabb();
    }
}
