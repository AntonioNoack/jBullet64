package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.VectorUtil;

import javax.vecmath.Vector3d;

/**
 * Allows accessing vertex data.
 *
 * @author jezek2
 */
public interface VertexData {

    int getVertexCount();

    int getIndexCount();

    Vector3d getVertex(int idx, Vector3d out);

    void setVertex(int idx, double x, double y, double z);

    @SuppressWarnings("unused")
    default void setVertex(int idx, Vector3d t) {
        setVertex(idx, t.x, t.y, t.z);
    }

    int getIndex(int idx);

    default void getTriangle(int firstIndex, Vector3d scale, Vector3d[] triangle) {
        for (int i = 0; i < 3; i++) {
            getVertex(getIndex(firstIndex + i), triangle[i]);
            VectorUtil.mul(triangle[i], triangle[i], scale);
        }
    }
}
