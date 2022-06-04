package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3d;

/**
 * SimplexSolverInterface can incrementally calculate distance between origin and
 * up to 4 vertices. Used by GJK or Linear Casting. Can be implemented by the
 * Johnson-algorithm or alternative approaches based on voronoi regions or barycentric
 * coordinates.
 * 
 * @author jezek2
 */
public abstract class SimplexSolverInterface {

	public abstract void reset();

	public abstract void addVertex(Vector3d w, Vector3d p, Vector3d q);
	
	public abstract boolean closest(Vector3d v);

	public abstract double maxVertex();

	public abstract boolean fullSimplex();

	public abstract int getSimplex(Vector3d[] pBuf, Vector3d[] qBuf, Vector3d[] yBuf);

	public abstract boolean inSimplex(Vector3d w);
	
	public abstract void backup_closest(Vector3d v);

	public abstract boolean emptySimplex();

	public abstract void compute_points(Vector3d p1, Vector3d p2);

	public abstract int numVertices();
	
}
