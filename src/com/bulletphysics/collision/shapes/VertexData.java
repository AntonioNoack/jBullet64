package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.VectorUtil;

import javax.vecmath.Vector3d;

/**
 * Allows accessing vertex data.
 *
 * @author jezek2
 */
public abstract class VertexData {

    public abstract int getVertexCount();

    public abstract int getIndexCount();

    public abstract <T extends Vector3d> T getVertex(int idx, T out);

    public abstract void setVertex(int idx, double x, double y, double z);

    public void setVertex(int idx, Vector3d t) {
        setVertex(idx, t.x, t.y, t.z);
    }

    public abstract int getIndex(int idx);

    public void getTriangle(int firstIndex, Vector3d scale, Vector3d[] triangle) {
        for (int i = 0; i < 3; i++) {
            getVertex(getIndex(firstIndex + i), triangle[i]);
            VectorUtil.mul(triangle[i], triangle[i], scale);
        }
    }

}
