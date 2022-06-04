package com.bulletphysics.collision.shapes;

import java.io.Serializable;

/**
 * BvhSubtreeInfo provides info to gather a subtree of limited size.
 * 
 * @author jezek2
 */
public class BvhSubtreeInfo implements Serializable {

	private static final long serialVersionUID = 1L;
	
	public final /*unsigned*/ short[] quantizedAabbMin = new short[3];
	public final /*unsigned*/ short[] quantizedAabbMax = new short[3];
	// points to the root of the subtree
	public int rootNodeIndex;
	public int subtreeSize;

	public void setAabbFromQuantizeNode(QuantizedBvhNodes quantizedNodes, int nodeId) {
		quantizedAabbMin[0] = (short)quantizedNodes.getQuantizedAabbMin(nodeId, 0);
		quantizedAabbMin[1] = (short)quantizedNodes.getQuantizedAabbMin(nodeId, 1);
		quantizedAabbMin[2] = (short)quantizedNodes.getQuantizedAabbMin(nodeId, 2);
		quantizedAabbMax[0] = (short)quantizedNodes.getQuantizedAabbMax(nodeId, 0);
		quantizedAabbMax[1] = (short)quantizedNodes.getQuantizedAabbMax(nodeId, 1);
		quantizedAabbMax[2] = (short)quantizedNodes.getQuantizedAabbMax(nodeId, 2);
	}

}
