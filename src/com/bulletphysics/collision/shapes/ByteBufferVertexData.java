/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

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
            throw new IllegalStateException("indicies type must be short or integer");
        }
    }

}
