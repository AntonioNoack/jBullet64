package com.bulletphysics.collision.shapes;

import javax.vecmath.Vector3d;
import java.nio.ByteBuffer;

/**
 * @author jezek2
 */
public class ByteBufferVertexData extends VertexData {

    public ByteBuffer vertexData;
    public int vertexCount;
    public int vertexStride;
    public ScalarType vertexType;

    public ByteBuffer indexData;
    public int indexCount;
    public int indexStride;
    public ScalarType indexType;

    @Override
    public int getVertexCount() {
        return vertexCount;
    }

    @Override
    public int getIndexCount() {
        return indexCount;
    }

    @Override
    public <T extends Vector3d> T getVertex(int idx, T out) {
        int off = idx * vertexStride;
        out.x = vertexData.getFloat(off);
        out.y = vertexData.getFloat(off + 4);
        out.z = vertexData.getFloat(off + 8);
        return out;
    }

    @Override
    public void setVertex(int idx, double x, double y, double z) {
        int off = idx * vertexStride;
        vertexData.putFloat(off, (float) x);
        vertexData.putFloat(off + 4, (float) y);
        vertexData.putFloat(off + 8, (float) z);
    }

    @Override
    public int getIndex(int idx) {
        if (indexType == ScalarType.SHORT) {
            return indexData.getShort(idx * indexStride) & 0xFFFF;
        } else if (indexType == ScalarType.INTEGER) {
            return indexData.getInt(idx * indexStride);
        } else {
            throw new IllegalStateException("indicies type must be short or integer");
        }
    }

}
