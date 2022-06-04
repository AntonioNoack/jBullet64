package com.bulletphysics.linearmath.convexhull;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

/**
 * Describes point cloud data and other input for conversion to polygonal representation.
 *
 * @author jezek2
 */
public class HullDesc {

    /**
     * Flags to use when generating the convex hull, see {@link HullFlags}.
     */
    public int flags = HullFlags.DEFAULT;

    /**
     * Number of vertices in the input point cloud.
     */
    public int vertexCount = 0;

    /**
     * Array of vertices.
     */
    public ArrayList<Vector3d> vertices;

    /**
     * Stride of each vertex, in bytes.
     */
    int vertexStride = 3 * 4;

    /**
     * Epsilon value for removing duplicates. This is a normalized value, if normalized bit is on.
     */
    public double normalEpsilon = 0.001;

    /**
     * Maximum number of vertices to be considered for the hull.
     */
    public int maxVertices = 4096;

    public HullDesc() {
    }

    public HullDesc(int flag, int vertexCount, ArrayList<Vector3d> vertices) {
        this(flag, vertexCount, vertices, 3 * 4);
    }

    public HullDesc(int flag, int vertexCount, ArrayList<Vector3d> vertices, int stride) {
        this.flags = flag;
        this.vertexCount = vertexCount;
        this.vertices = vertices;
        this.vertexStride = stride;
    }

    public boolean hasHullFlag(int flag) {
        return (flags & flag) != 0;
    }

    public void setHullFlag(int flag) {
        flags |= flag;
    }

    public void clearHullFlag(int flag) {
        flags &= ~flag;
    }

}
