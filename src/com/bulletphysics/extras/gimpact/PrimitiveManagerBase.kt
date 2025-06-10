package com.bulletphysics.extras.gimpact;

import com.bulletphysics.extras.gimpact.BoxCollision.AABB;

/**
 * Prototype Base class for primitive classification.<p>
 * 
 * This class is a wrapper for primitive collections.<p>
 * 
 * This tells relevant info for the Bounding Box set classes, which take care of space classification.<p>
 * 
 * This class can manage Compound shapes and trimeshes, and if it is managing trimesh then the
 * Hierarchy Bounding Box classes will take advantage of primitive Vs Box overlapping tests for
 * getting optimal results and less Per Box compairisons.
 * 
 * @author jezek2
 */
abstract class PrimitiveManagerBase {

	/**
	 * Determines if this manager consist on only triangles, which special case will be optimized.
	 */
	public abstract boolean isTrimesh();

	public abstract int getPrimitiveCount();

	public abstract void getPrimitiveBox(int primitiveIndex, AABB dst);
	
	/**
	 * Retrieves only the points of the triangle, and the collision margin.
	 */
	public abstract void getPrimitiveTriangle(int prim_index, PrimitiveTriangle triangle);
	
}
