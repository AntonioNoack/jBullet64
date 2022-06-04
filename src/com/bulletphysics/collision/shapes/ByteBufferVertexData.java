package com.bulletphysics.collision.shapes;

import javax.vecmath.Tuple3d;
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
    public <T extends Tuple3d> T getVertex(int idx, T out) {
        int off = idx * vertexStride;
        ByteBuffer vertexData = this.vertexData;
        boolean useFloats = vertexStride < 24;
        if (off < 0 || off + (useFloats ? 4 : 8) * 3 > vertexData.limit()) {
            throw new IndexOutOfBoundsException("Vertex Index " + idx + " * Stride " + vertexStride +
                    " += " + ((vertexStride < 24 ? 4 : 8) * 3) +
                    " !in 0 until limit/capacity " + vertexData.limit() + "/" + vertexData.capacity());
        }
        if (useFloats) {// can only be floats
            out.x = vertexData.getFloat(off);
            out.y = vertexData.getFloat(off + 4);
            out.z = vertexData.getFloat(off + 8);
        } else {
            out.x = vertexData.getDouble(off);
            out.y = vertexData.getDouble(off + 8);
            out.z = vertexData.getDouble(off + 16);
        }
        return out;
    }

    @Override
    public void setVertex(int idx, double x, double y, double z) {
        int off = idx * vertexStride;
        if (vertexStride < 24) {// can only be floats
            vertexData.putFloat(off, (float) x);
            vertexData.putFloat(off + 4, (float) y);
            vertexData.putFloat(off + 8, (float) z);
        } else {
            vertexData.putDouble(off, x);
            vertexData.putDouble(off + 8, y);
            vertexData.putDouble(off + 16, z);
        }
    }

    @Override
    public int getIndex(int idx) {
        if (indexType == ScalarType.SHORT) {
            return indexData.getShort(idx * indexStride) & 0xFFFF;
        } else if (indexType == ScalarType.INTEGER) {
            return indexData.getInt(idx * indexStride);
        } else {
            throw new IllegalStateException("indices type must be short or integer");
        }
    }

}
