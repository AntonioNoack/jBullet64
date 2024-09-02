package com.bulletphysics.collision.shapes;

import com.bulletphysics.util.ObjectArrayList;
import java.nio.ByteBuffer;

/**
 * TriangleIndexVertexArray allows to use multiple meshes, by indexing into existing
 * triangle/index arrays. Additional meshes can be added using {@link #addIndexedMesh addIndexedMesh}.<p>
 * 
 * No duplicate is made of the vertex/index data, it only indexes into external vertex/index
 * arrays. So keep those arrays around during the lifetime of this TriangleIndexVertexArray.
 * 
 * @author jezek2
 */
public class TriangleIndexVertexArray extends StridingMeshInterface {

	protected ObjectArrayList<IndexedMesh> indexedMeshes = new ObjectArrayList<IndexedMesh>();

	private final ByteBufferVertexData data = new ByteBufferVertexData();

	public TriangleIndexVertexArray() {
	}

	/**
	 * Just to be backwards compatible.
	 */
	public TriangleIndexVertexArray(int numTriangles, ByteBuffer triangleIndexBase, int triangleIndexStride, int numVertices, ByteBuffer vertexBase, int vertexStride) {
		IndexedMesh mesh = new IndexedMesh();

		mesh.numTriangles = numTriangles;
		mesh.triangleIndexBase = triangleIndexBase;
		mesh.triangleIndexStride = triangleIndexStride;
		mesh.numVertices = numVertices;
		mesh.vertexBase = vertexBase;
		mesh.vertexStride = vertexStride;

		addIndexedMesh(mesh);
	}

	public void addIndexedMesh(IndexedMesh mesh) {
		addIndexedMesh(mesh, ScalarType.INTEGER);
	}

	public void addIndexedMesh(IndexedMesh mesh, ScalarType indexType) {
		indexedMeshes.add(mesh);
		indexedMeshes.getQuick(indexedMeshes.size() - 1).indexType = indexType;
	}
	
	@Override
	public VertexData getLockedVertexIndexBase(int subpart) {
		assert (subpart < getNumSubParts());

		IndexedMesh mesh = indexedMeshes.getQuick(subpart);

		data.vertexCount = mesh.numVertices;
		data.vertexData = mesh.vertexBase;
		//#ifdef BT_USE_DOUBLE_PRECISION
		//type = PHY_DOUBLE;
		//#else
		data.vertexType = ScalarType.FLOAT;
		//#endif
		data.vertexStride = mesh.vertexStride;

		data.indexCount = mesh.numTriangles*3;

		data.indexData = mesh.triangleIndexBase;
		data.indexStride = mesh.triangleIndexStride/3;
		data.indexType = mesh.indexType;
		return data;
	}

	@Override
	public VertexData getLockedReadOnlyVertexIndexBase(int subpart) {
		return getLockedVertexIndexBase(subpart);
	}

	/**
	 * unLockVertexBase finishes the access to a subpart of the triangle mesh.
	 * Make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished.
	 */
	@Override
	public void unLockVertexBase(int subpart) {
		data.vertexData = null;
		data.indexData = null;
	}

	@Override
	public void unLockReadOnlyVertexBase(int subpart) {
		unLockVertexBase(subpart);
	}

	/**
	 * getNumSubParts returns the number of seperate subparts.
	 * Each subpart has a continuous array of vertices and indices.
	 */
	@Override
	public int getNumSubParts() {
		return indexedMeshes.size();
	}

	public ObjectArrayList<IndexedMesh> getIndexedMeshArray() {
		return indexedMeshes;
	}
	
	@Override
	public void preallocateVertices(int numVertices) {
	}

	@Override
	public void preallocateIndices(int numIndices) {
	}

}
