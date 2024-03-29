package com.bulletphysics.linearmath.convexhull;

import com.bulletphysics.util.IntArrayList;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

/**
 * Contains resulting polygonal representation.<p>
 * <p>
 * Depending on the {@link #polygons} flag, array of indices consists of:<br>
 * <b>for triangles:</b> indices are array indexes into the vertex list<br>
 * <b>for polygons:</b> indices are in the form (number of points in face) (p1, p2, p3, ...)
 *
 * @author jezek2
 */
public class HullResult {

    /**
     * True if indices represents polygons, false indices are triangles.
     */
    public boolean polygons = true;

    public int numOutputVertices = 0;

    /**
     * Array of vertices.
     */
    public final ArrayList<Vector3d> outputVertices = new ArrayList<>();

    public int numFaces = 0;

    public int numIndices = 0;

    public final IntArrayList indices = new IntArrayList();

}
