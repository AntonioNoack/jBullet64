package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.StridingMeshInterface;
import com.bulletphysics.collision.shapes.VertexData;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.VectorUtil;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
class TrimeshPrimitiveManager extends PrimitiveManagerBase {

    public double margin;
    public StridingMeshInterface meshInterface;
    public final Vector3d scale = new Vector3d();
    public int part;
    public int lock_count;

    private final int[] tmpIndices = new int[3];

    private VertexData vertexData;

    public TrimeshPrimitiveManager() {
        meshInterface = null;
        part = 0;
        margin = 0.01f;
        scale.set(1.0, 1.0, 1.0);
        lock_count = 0;
    }

    public TrimeshPrimitiveManager(TrimeshPrimitiveManager manager) {
        meshInterface = manager.meshInterface;
        part = manager.part;
        margin = manager.margin;
        scale.set(manager.scale);
        lock_count = 0;
    }

    public TrimeshPrimitiveManager(StridingMeshInterface meshInterface, int part) {
        this.meshInterface = meshInterface;
        this.part = part;
        this.meshInterface.getScaling(scale);
        margin = 0.1;
        lock_count = 0;
    }

    public void lock() {
        if (lock_count > 0) {
            lock_count++;
            return;
        }
        vertexData = meshInterface.getLockedReadOnlyVertexIndexBase(part);

        lock_count = 1;
    }

    public void unlock() {
        if (lock_count == 0) {
            return;
        }
        if (lock_count > 1) {
            --lock_count;
            return;
        }
        meshInterface.unLockReadOnlyVertexBase(part);
        vertexData = null;
        lock_count = 0;
    }

    @Override
    public boolean isTrimesh() {
        return true;
    }

    @Override
    public int getPrimitiveCount() {
        return vertexData.getIndexCount() / 3;
    }

    public int getVertexCount() {
        return vertexData.getVertexCount();
    }

    public void getIndices(int face_index, int[] out) {
        out[0] = vertexData.getIndex(face_index * 3);
        out[1] = vertexData.getIndex(face_index * 3 + 1);
        out[2] = vertexData.getIndex(face_index * 3 + 2);
    }

    public void getVertex(int vertex_index, Vector3d vertex) {
        vertexData.getVertex(vertex_index, vertex);
        VectorUtil.mul(vertex, vertex, scale);
    }

    @Override
    public void getPrimitiveBox(int prim_index, AABB primbox) {
        PrimitiveTriangle triangle = new PrimitiveTriangle();
        getPrimitiveTriangle(prim_index, triangle);
        primbox.calcFromTriangleMargin(
                triangle.vertices[0],
                triangle.vertices[1], triangle.vertices[2], triangle.margin);
    }

    @Override
    public void getPrimitiveTriangle(int prim_index, PrimitiveTriangle triangle) {
        getIndices(prim_index, tmpIndices);
        getVertex(tmpIndices[0], triangle.vertices[0]);
        getVertex(tmpIndices[1], triangle.vertices[1]);
        getVertex(tmpIndices[2], triangle.vertices[2]);
        triangle.margin = margin;
    }

    public void getBulletTriangle(int prim_index, TriangleShapeEx triangle) {
        getIndices(prim_index, tmpIndices);
        getVertex(tmpIndices[0], triangle.vertices1[0]);
        getVertex(tmpIndices[1], triangle.vertices1[1]);
        getVertex(tmpIndices[2], triangle.vertices1[2]);
        triangle.setMargin(margin);
    }

}
