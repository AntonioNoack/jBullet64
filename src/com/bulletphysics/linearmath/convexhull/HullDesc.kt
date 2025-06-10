package com.bulletphysics.linearmath.convexhull;

import com.bulletphysics.util.ObjectArrayList;

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
    public int vcount = 0;

    /**
     * Array of vertices.
     */
    public ObjectArrayList<Vector3d> vertices;

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

    public HullDesc(int flag, int vcount, ObjectArrayList<Vector3d> vertices) {
        this.flags = flag;
        this.vcount = vcount;
        this.vertices = vertices;
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
