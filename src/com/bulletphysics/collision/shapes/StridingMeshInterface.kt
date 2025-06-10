package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * StridingMeshInterface is the abstract class for high performance access to
 * triangle meshes. It allows for sharing graphics and collision meshes.
 * Also, it provides locking/unlocking of graphics meshes that are in GPU memory.
 *
 * @author jezek2
 */
public abstract class StridingMeshInterface {

    protected final Vector3d scaling = new Vector3d(1.0, 1.0, 1.0);

    public void internalProcessAllTriangles(InternalTriangleIndexCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
        int graphicsSubParts = getNumSubParts();
        Vector3d[] triangle = new Vector3d[]{Stack.newVec(), Stack.newVec(), Stack.newVec()};

        Vector3d meshScaling = getScaling(Stack.newVec());

        for (int part = 0; part < graphicsSubParts; part++) {
            VertexData data = getLockedReadOnlyVertexIndexBase(part);
            for (int i = 0, cnt = data.indexCount / 3; i < cnt; i++) {
                data.getTriangle(i * 3, meshScaling, triangle);
                callback.internalProcessTriangleIndex(triangle, part, i);
            }
            unLockReadOnlyVertexBase(part);
        }
    }

    private static class AabbCalculationCallback implements InternalTriangleIndexCallback {
        public final Vector3d aabbMin = new Vector3d(1e308, 1e308, 1e308);
        public final Vector3d aabbMax = new Vector3d(-1e308, -1e308, -1e308);

        public void internalProcessTriangleIndex(Vector3d[] triangle, int partId, int triangleIndex) {
            VectorUtil.setMin(aabbMin, triangle[0]);
            VectorUtil.setMax(aabbMax, triangle[0]);
            VectorUtil.setMin(aabbMin, triangle[1]);
            VectorUtil.setMax(aabbMax, triangle[1]);
            VectorUtil.setMin(aabbMin, triangle[2]);
            VectorUtil.setMax(aabbMax, triangle[2]);
        }
    }

    public void calculateAabbBruteForce(Vector3d aabbMin, Vector3d aabbMax) {
        // first calculate the total aabb for all triangles
        AabbCalculationCallback aabbCallback = new AabbCalculationCallback();
        aabbMin.set(-1e308, -1e308, -1e308);
        aabbMax.set(1e308, 1e308, 1e308);
        internalProcessAllTriangles(aabbCallback, aabbMin, aabbMax);

        aabbMin.set(aabbCallback.aabbMin);
        aabbMax.set(aabbCallback.aabbMax);
    }

    /**
     * Get read and write access to a subpart of a triangle mesh.
     * This subpart has a continuous array of vertices and indices.
     * In this way the mesh can be handled as chunks of memory with striding
     * very similar to OpenGL vertexarray support.
     * Make a call to unLockVertexBase when the read and write access is finished.
     */
    public abstract VertexData getLockedVertexIndexBase(int subpart/*=0*/);

    public abstract VertexData getLockedReadOnlyVertexIndexBase(int subpart/*=0*/);

    /**
     * unLockVertexBase finishes the access to a subpart of the triangle mesh.
     * Make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished.
     */
    public abstract void unLockVertexBase(int subpart);

    public abstract void unLockReadOnlyVertexBase(int subpart);

    /**
     * getNumSubParts returns the number of seperate subparts.
     * Each subpart has a continuous array of vertices and indices.
     */
    public abstract int getNumSubParts();

    public abstract void preallocateVertices(int numVertices);

    public abstract void preallocateIndices(int numIndices);

    public Vector3d getScaling(Vector3d out) {
        out.set(scaling);
        return out;
    }

    public void setScaling(Vector3d scaling) {
        this.scaling.set(scaling);
    }

}
