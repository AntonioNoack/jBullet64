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
public interface SimplexSolverInterface {

	void reset();

	void addVertex(Vector3d w, Vector3d p, Vector3d q);
	
	boolean closest(Vector3d v);

	double maxVertex();

	boolean fullSimplex();

	int getSimplex(Vector3d[] pBuf, Vector3d[] qBuf, Vector3d[] yBuf);

	boolean inSimplex(Vector3d w);
	
	void backupClosest(Vector3d v);

	boolean emptySimplex();

	void computePoints(Vector3d p1, Vector3d p2);

	int numVertices();
	
}
