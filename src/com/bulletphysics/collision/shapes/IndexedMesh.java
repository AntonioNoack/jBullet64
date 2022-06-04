package com.bulletphysics.collision.shapes;

import java.nio.ByteBuffer;

/**
 * IndexedMesh indexes into existing vertex and index arrays, in a similar way to
 * OpenGL's glDrawElements. Instead of the number of indices, we pass the number
 * of triangles.
 * 
 * @author jezek2
 */
public class IndexedMesh {
	
	public int numTriangles;
	public ByteBuffer triangleIndexBase;
	public int triangleIndexStride;
	public int numVertices;
	public ByteBuffer vertexBase;
	public int vertexStride;
	// The index type is set when adding an indexed mesh to the
	// TriangleIndexVertexArray, do not set it manually
	public ScalarType indexType;

}
