package com.bulletphysics.collision.shapes;

/**
 * Callback for operating with {@link OptimizedBvh}.
 * 
 * @author jezek2
 */
public interface NodeOverlapCallback {

	void processNode(int subPart, int triangleIndex);
	
}
