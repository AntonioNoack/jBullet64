package com.bulletphysics.collision.shapes;

import cz.advel.stack.Stack;

import java.io.Serializable;
import javax.vecmath.Vector3d;

/**
 * OptimizedBvhNode contains both internal and leaf node information.
 * 
 * @author jezek2
 */
public class OptimizedBvhNode implements Serializable {

	private static final long serialVersionUID = 1L;
	
	public final Vector3d aabbMinOrg = new Vector3d();
	public final Vector3d aabbMaxOrg = new Vector3d();

	public int escapeIndex;

	// for child nodes
	public int subPart;
	public int triangleIndex;
	
	public void set(OptimizedBvhNode n) {
		aabbMinOrg.set(n.aabbMinOrg);
		aabbMaxOrg.set(n.aabbMaxOrg);
		escapeIndex = n.escapeIndex;
		subPart = n.subPart;
		triangleIndex = n.triangleIndex;
	}

}
