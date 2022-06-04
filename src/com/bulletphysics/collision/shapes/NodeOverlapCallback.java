package com.bulletphysics.collision.shapes;

/**
 * Callback for operating with {@link OptimizedBvh}.
 * 
 * @author jezek2
 */
public abstract class NodeOverlapCallback {

	public abstract void processNode(int subPart, int triangleIndex);
	
}
