package com.bulletphysics.linearmath.convexhull;

/**
 * Flags that affects convex hull generation, used in {@link HullDesc#flags}.
 * 
 * @author jezek2
 */
public class HullFlags {
	
	public static int TRIANGLES     = 1; // report results as triangles, not polygons.
	public static int REVERSE_ORDER = 2; // reverse order of the triangle indices.
	public static int DEFAULT       = TRIANGLES;
	
}
