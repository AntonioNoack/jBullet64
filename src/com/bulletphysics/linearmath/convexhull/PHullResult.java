package com.bulletphysics.linearmath.convexhull;

import com.bulletphysics.util.IntArrayList;
import java.util.ArrayList;
import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
class PHullResult {
	
	public int vertexCount = 0;
	public int indexCount = 0;
	public int faceCount = 0;
	public ArrayList<Vector3d> vertices = null;
	public IntArrayList indices = new IntArrayList();
	
}
