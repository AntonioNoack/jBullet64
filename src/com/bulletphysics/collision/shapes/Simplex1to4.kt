package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;

import javax.vecmath.Vector3d;

/**
 * implements feature based and implicit simplex of up to 4 vertices
 * (tetrahedron, triangle, line, vertex).
 * 
 * @author jezek2
 */
public class Simplex1to4 extends PolyhedralConvexShape {

	int numVertices = 0;
	Vector3d[] vertices = new Vector3d[4];

	public Simplex1to4() {
	}

	@SuppressWarnings("unused")
	public Simplex1to4(Vector3d pt0) {
		addVertex(pt0);
	}

	@SuppressWarnings("unused")
	public Simplex1to4(Vector3d pt0, Vector3d pt1) {
		addVertex(pt0);
		addVertex(pt1);
	}

	@SuppressWarnings("unused")
	public Simplex1to4(Vector3d pt0, Vector3d pt1, Vector3d pt2) {
		addVertex(pt0);
		addVertex(pt1);
		addVertex(pt2);
	}

	@SuppressWarnings("unused")
	public Simplex1to4(Vector3d pt0, Vector3d pt1, Vector3d pt2, Vector3d pt3) {
		addVertex(pt0);
		addVertex(pt1);
		addVertex(pt2);
		addVertex(pt3);
	}
	
	public void reset() {
		numVertices = 0;
	}
	
	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.TETRAHEDRAL_SHAPE_PROXYTYPE;
	}
	
	public void addVertex(Vector3d pt) {
		vertices[numVertices++] = pt;
		recalculateLocalAabb();
	}

	
	@Override
	public int getNumVertices() {
		return numVertices;
	}

	@Override
	public int getNumEdges() {
		// euler formula, F-E+V = 2, so E = F+V-2

		switch (numVertices) {
			case 2: return 1;
			case 3: return 3;
			case 4: return 6;
		}

		return 0;
	}

	@Override
	public void getEdge(int i, Vector3d pa, Vector3d pb) {
		switch (numVertices) {
			case 2:
				pa.set(vertices[0]);
				pb.set(vertices[1]);
				break;
			case 3:
				switch (i) {
					case 0:
						pa.set(vertices[0]);
						pb.set(vertices[1]);
						break;
					case 1:
						pa.set(vertices[1]);
						pb.set(vertices[2]);
						break;
					case 2:
						pa.set(vertices[2]);
						pb.set(vertices[0]);
						break;
				}
				break;
			case 4:
				switch (i) {
					case 0:
						pa.set(vertices[0]);
						pb.set(vertices[1]);
						break;
					case 1:
						pa.set(vertices[1]);
						pb.set(vertices[2]);
						break;
					case 2:
						pa.set(vertices[2]);
						pb.set(vertices[0]);
						break;
					case 3:
						pa.set(vertices[0]);
						pb.set(vertices[3]);
						break;
					case 4:
						pa.set(vertices[1]);
						pb.set(vertices[3]);
						break;
					case 5:
						pa.set(vertices[2]);
						pb.set(vertices[3]);
						break;
				}
		}
	}

	@Override
	public void getVertex(int i, Vector3d vtx) {
		vtx.set(vertices[i]);
	}

	@Override
	public int getNumPlanes() {
		switch (numVertices) {
			case 3: return 2;
			case 4: return 4;
		}
		return 0;
	}

	@Override
	public void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i) {
	}

	@Override
	public boolean isInside(Vector3d pt, double tolerance) {
		return false;
	}
}
