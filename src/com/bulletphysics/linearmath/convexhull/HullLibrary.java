
// includes modifications/improvements by John Ratcliff, see BringOutYourDead below.
package com.bulletphysics.linearmath.convexhull;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.IntArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * HullLibrary class can create a convex hull from a collection of vertices, using
 * the ComputeHull method. The {@link ShapeHull} class uses this HullLibrary to create
 * a approximate convex mesh given a general (non-polyhedral) convex shape.
 *
 * @author jezek2
 */
public class HullLibrary {

	public final IntArrayList vertexIndexMapping = new IntArrayList();

	private final ArrayList<Tri> triangles = new ArrayList<>();

	/**
	 * Converts point cloud to polygonal representation.
	 *
	 * @param desc   describes the input request
	 * @param result contains the result
	 * @return whether conversion was successful
	 */
	public boolean createConvexHull(HullDesc desc, HullResult result) {
		boolean ret = false;

		PHullResult hr = new PHullResult();

		int vcount = desc.vertexCount;
		if (vcount < 8) vcount = 8;

		ArrayList<Vector3d> vertexSource = new ArrayList<>(vcount);
		for (int i = 0; i < vcount; i++) vertexSource.add(new Vector3d());

		Vector3d scale = Stack.newVec();

		int[] oldVertexCount = new int[1];
		boolean ok = cleanupVertices(desc.vertexCount, desc.vertices, desc.vertexStride, oldVertexCount, vertexSource, desc.normalEpsilon, scale); // normalize point cloud, remove duplicates!

		if (ok) {
			//		if ( 1 ) // scale vertices back to their original size.
			{
				for (int i = 0; i < oldVertexCount[0]; i++) {
					Vector3d v = vertexSource.get(i);
					VectorUtil.mul(v, v, scale);
				}
			}

			ok = computeHull(oldVertexCount[0], vertexSource, hr, desc.maxVertices);

			if (ok) {
				// re-index triangle mesh, so it refers to only used vertices, rebuild a new vertex table.
				ArrayList<Vector3d> vertexScratch = new ArrayList<>();
				for (int i = 0, l = hr.vertexCount; i < l; i++) vertexScratch.add(new Vector3d());

				bringOutYourDead(hr.vertices, hr.vertexCount, vertexScratch, oldVertexCount, hr.indices, hr.indexCount);

				ret = true;

				if (desc.hasHullFlag(HullFlags.TRIANGLES)) { // if he wants the results as triangle!
					result.polygons = false;
					result.numOutputVertices = oldVertexCount[0];
					MiscUtil.resize(result.outputVertices, oldVertexCount[0], Vector3d.class);
					result.numFaces = hr.faceCount;
					result.numIndices = hr.indexCount;

					MiscUtil.resize(result.indices, hr.indexCount);

					for (int i = 0; i < oldVertexCount[0]; i++) {
						result.outputVertices.get(i).set(vertexScratch.get(i));
					}

					if (desc.hasHullFlag(HullFlags.REVERSE_ORDER)) {
						IntArrayList source_ptr = hr.indices;
						int srcIdx = 0;

						IntArrayList destPtr = result.indices;
						int dstIdx = 0;

						for (int i = 0; i < hr.faceCount; i++) {
							destPtr.set(dstIdx, source_ptr.get(srcIdx + 2));
							destPtr.set(dstIdx + 1, source_ptr.get(srcIdx + 1));
							destPtr.set(dstIdx + 2, source_ptr.get(srcIdx));
							dstIdx += 3;
							srcIdx += 3;
						}
					} else {
						for (int i = 0; i < hr.indexCount; i++) {
							result.indices.set(i, hr.indices.get(i));
						}
					}
				} else {
					result.polygons = true;
					result.numOutputVertices = oldVertexCount[0];
					MiscUtil.resize(result.outputVertices, oldVertexCount[0], Vector3d.class);
					result.numFaces = hr.faceCount;
					result.numIndices = hr.indexCount + hr.faceCount;
					MiscUtil.resize(result.indices, result.numIndices);
					for (int i = 0; i < oldVertexCount[0]; i++) {
						result.outputVertices.get(i).set(vertexScratch.get(i));
					}

					//				if ( 1 )
					{
						IntArrayList srcPtr = hr.indices;
						int srcIdx = 0;

						IntArrayList dstPtr = result.indices;
						int dstIdx = 0;

						for (int i = 0; i < hr.faceCount; i++) {
							dstPtr.set(dstIdx, 3);
							if (desc.hasHullFlag(HullFlags.REVERSE_ORDER)) {
								dstPtr.set(dstIdx + 1, srcPtr.get(srcIdx + 2));
								dstPtr.set(dstIdx + 2, srcPtr.get(srcIdx + 1));
								dstPtr.set(dstIdx + 3, srcPtr.get(srcIdx));
							} else {
								dstPtr.set(dstIdx + 1, srcPtr.get(srcIdx));
								dstPtr.set(dstIdx + 2, srcPtr.get(srcIdx + 1));
								dstPtr.set(dstIdx + 3, srcPtr.get(srcIdx + 2));
							}

							dstIdx += 4;
							srcIdx += 3;
						}
					}
				}
				releaseHull(hr);
			}
		}

		return ret;
	}

	/**
	 * Release memory allocated for this result, we are done with it.
	 */
	public void releaseResult(HullResult result) {
		if (result.outputVertices.size() != 0) {
			result.numOutputVertices = 0;
			result.outputVertices.clear();
		}
		if (result.indices.size() != 0) {
			result.numIndices = 0;
			result.indices.clear();
		}
	}

	private boolean computeHull(int vertexCount, ArrayList<Vector3d> vertices, PHullResult result, int vertexLimit) {
		int[] tris_count = new int[1];
		int ret = calcHull(vertices, vertexCount, result.indices, tris_count, vertexLimit);
		if (ret == 0) return false;
		result.indexCount = tris_count[0] * 3;
		result.faceCount = tris_count[0];
		result.vertices = vertices;
		result.vertexCount = vertexCount;
		return true;
	}

	private Tri allocateTriangle(int a, int b, int c) {
		Tri tr = new Tri(a, b, c);
		tr.id = triangles.size();
		triangles.add(tr);

		return tr;
	}

	private void deAllocateTriangle(Tri tri) {
		assert (triangles.get(tri.id) == tri);
		triangles.set(tri.id, null);
	}

	private void fixB2B(Tri s, Tri t) {
		for (int i = 0; i < 3; i++) {
			int i1 = (i + 1) % 3;
			int i2 = (i + 2) % 3;
			int a = s.getCoord(i1);
			int b = s.getCoord(i2);
			assert (triangles.get(s.getNeighbor(a, b)).getNeighbor(b, a) == s.id);
			assert (triangles.get(t.getNeighbor(a, b)).getNeighbor(b, a) == t.id);
			triangles.get(s.getNeighbor(a, b)).setNeighbor(b, a, t.getNeighbor(b, a));
			triangles.get(t.getNeighbor(b, a)).setNeighbor(a, b, s.getNeighbor(a, b));
		}
	}

	private void removeB2B(Tri s, Tri t) {
		fixB2B(s, t);
		deAllocateTriangle(s);
		deAllocateTriangle(t);
	}

	private void checkIt(Tri t) {
		assert (triangles.get(t.id) == t);
		for (int i = 0; i < 3; i++) {
			int i1 = (i + 1) % 3;
			int i2 = (i + 2) % 3;
			int a = t.getCoord(i1);
			int b = t.getCoord(i2);

			assert (a != b);
			assert (triangles.get(t.n.getCoord(i)).getNeighbor(b, a) == t.id);
		}
	}

	private Tri extrudable(double epsilon) {
		Tri t = null;
		for (Tri triangle : triangles) {
			if (t == null || (triangle != null && t.rise < triangle.rise)) {
				t = triangle;
			}
		}
		assert t != null;
		return (t.rise > epsilon) ? t : null;
	}

	private int calcHull(ArrayList<Vector3d> vertices, int vertexCount, IntArrayList tris_out, int[] tris_count, int vertexLimit) {
		int rc = calcHullGen(vertices, vertexCount, vertexLimit);
		if (rc == 0) return 0;

		IntArrayList ts = new IntArrayList();

		for (Tri triangle : triangles) {
			if (triangle != null) {
				for (int j = 0; j < 3; j++) {
					ts.add(triangle.getCoord(j));
				}
				deAllocateTriangle(triangle);
			}
		}
		tris_count[0] = ts.size() / 3;
		MiscUtil.resize(tris_out, ts.size());

		for (int i = 0; i < ts.size(); i++) {
			tris_out.set(i, ts.get(i));
		}
		triangles.clear();

		return 1;
	}

	private int calcHullGen(ArrayList<Vector3d> vertices, int vertexCount, int vertexLimit) {
		if (vertexCount < 4) return 0;

		Vector3d tmp = Stack.newVec();
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();

		if (vertexLimit == 0) {
			vertexLimit = 1000000000;
		}
		//int j;
		Vector3d bmin = Stack.newVec(vertices.get(0));
		Vector3d bmax = Stack.newVec(vertices.get(0));
		IntArrayList isextreme = new IntArrayList();
		//isextreme.reserve(verts_count);
		IntArrayList allow = new IntArrayList();
		//allow.reserve(verts_count);

		for (int j = 0; j < vertexCount; j++) {
			allow.add(1);
			isextreme.add(0);
			VectorUtil.setMin(bmin, vertices.get(j));
			VectorUtil.setMax(bmax, vertices.get(j));
		}
		tmp.sub(bmax, bmin);
		double epsilon = tmp.length() * 0.001;
		assert (epsilon != 0.0);

		Int4 p = findSimplex(vertices, vertexCount, allow, new Int4());
		if (p.x == -1) {
			return 0; // simplex failed

			// a valid interior point
		}
		Vector3d center = Stack.newVec();
		VectorUtil.add(center, vertices.get(p.getCoord(0)), vertices.get(p.getCoord(1)), vertices.get(p.getCoord(2)), vertices.get(p.getCoord(3)));
		center.scale(0.25);

		Tri t0 = allocateTriangle(p.getCoord(2), p.getCoord(3), p.getCoord(1));
		t0.n.set(2, 3, 1);
		Tri t1 = allocateTriangle(p.getCoord(3), p.getCoord(2), p.getCoord(0));
		t1.n.set(3, 2, 0);
		Tri t2 = allocateTriangle(p.getCoord(0), p.getCoord(1), p.getCoord(3));
		t2.n.set(0, 1, 3);
		Tri t3 = allocateTriangle(p.getCoord(1), p.getCoord(0), p.getCoord(2));
		t3.n.set(1, 0, 2);
		isextreme.set(p.getCoord(0), 1);
		isextreme.set(p.getCoord(1), 1);
		isextreme.set(p.getCoord(2), 1);
		isextreme.set(p.getCoord(3), 1);
		checkIt(t0);
		checkIt(t1);
		checkIt(t2);
		checkIt(t3);

		Vector3d n = Stack.newVec();

		for (Tri t : triangles) {
			assert (t != null);
			assert (t.maxValue < 0);
			triNormal(vertices.get(t.getCoord(0)), vertices.get(t.getCoord(1)), vertices.get(t.getCoord(2)), n);
			t.maxValue = maxdirsterid(vertices, vertexCount, n, allow);
			tmp.sub(vertices.get(t.maxValue), vertices.get(t.getCoord(0)));
			t.rise = n.dot(tmp);
		}
		Tri te;
		vertexLimit -= 4;
		while (vertexLimit > 0 && ((te = extrudable(epsilon)) != null)) {
			int v = te.maxValue;
			assert (v != -1);
			assert (isextreme.get(v) == 0);  // wtf we've already done this vertex
			isextreme.set(v, 1);
			//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
			int j = triangles.size();
			while ((j--) != 0) {
				if (triangles.get(j) == null) {
					continue;
				}
				Int3 t = triangles.get(j);
				if (above(vertices, t, vertices.get(v), 0.01 * epsilon)) {
					extrude(triangles.get(j), v);
				}
			}
			// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
			j = triangles.size();
			while ((j--) != 0) {
				if (triangles.get(j) == null) {
					continue;
				}
				if (!hasVertex(triangles.get(j), v)) {
					break;
				}
				Int3 nt = triangles.get(j);
				tmp1.sub(vertices.get(nt.getCoord(1)), vertices.get(nt.getCoord(0)));
				tmp2.sub(vertices.get(nt.getCoord(2)), vertices.get(nt.getCoord(1)));
				tmp.cross(tmp1, tmp2);
				if (above(vertices, nt, center, 0.01 * epsilon) || tmp.length() < epsilon * epsilon * 0.1) {
					Tri nb = triangles.get(triangles.get(j).n.getCoord(0));
					assert (nb != null);
					assert (!hasVertex(nb, v));
					assert (nb.id < j);
					extrude(nb, v);
					j = triangles.size();
				}
			}
			j = triangles.size();
			while ((j--) != 0) {
				Tri t = triangles.get(j);
				if (t == null) {
					continue;
				}
				if (t.maxValue >= 0) {
					break;
				}
				triNormal(vertices.get(t.getCoord(0)), vertices.get(t.getCoord(1)), vertices.get(t.getCoord(2)), n);
				t.maxValue = maxdirsterid(vertices, vertexCount, n, allow);
				if (isextreme.get(t.maxValue) != 0) {
					t.maxValue = -1; // already done that vertex - algorithm needs to be able to terminate.
				} else {
					tmp.sub(vertices.get(t.maxValue), vertices.get(t.getCoord(0)));
					t.rise = n.dot(tmp);
				}
			}
			vertexLimit--;
		}
		return 1;
	}

	private Int4 findSimplex(ArrayList<Vector3d> vertices, int vertexCount, IntArrayList allow, Int4 out) {
		Vector3d tmp = Stack.newVec();
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();

		Vector3d[] basis = new Vector3d[/*3*/]{Stack.newVec(), Stack.newVec(), Stack.newVec()};
		basis[0].set(0.01, 0.02, 1.0);
		int p0 = maxdirsterid(vertices, vertexCount, basis[0], allow);
		tmp.negate(basis[0]);
		int p1 = maxdirsterid(vertices, vertexCount, tmp, allow);
		basis[0].sub(vertices.get(p0), vertices.get(p1));
		if (p0 == p1 || (basis[0].x == 0.0 && basis[0].y == 0.0 && basis[0].z == 0.0)) {
			out.set(-1, -1, -1, -1);
			return out;
		}
		tmp.set(1f, 0.02, 0.0);
		basis[1].cross(tmp, basis[0]);
		tmp.set(-0.02, 1.0, 0.0);
		basis[2].cross(tmp, basis[0]);
		if (basis[1].length() > basis[2].length()) {
			basis[1].normalize();
		} else {
			basis[1].set(basis[2]);
			basis[1].normalize();
		}
		int p2 = maxdirsterid(vertices, vertexCount, basis[1], allow);
		if (p2 == p0 || p2 == p1) {
			tmp.negate(basis[1]);
			p2 = maxdirsterid(vertices, vertexCount, tmp, allow);
		}
		if (p2 == p0 || p2 == p1) {
			out.set(-1, -1, -1, -1);
			return out;
		}
		basis[1].sub(vertices.get(p2), vertices.get(p0));
		basis[2].cross(basis[1], basis[0]);
		basis[2].normalize();
		int p3 = maxdirsterid(vertices, vertexCount, basis[2], allow);
		if (p3 == p0 || p3 == p1 || p3 == p2) {
			tmp.negate(basis[2]);
			p3 = maxdirsterid(vertices, vertexCount, tmp, allow);
		}
		if (p3 == p0 || p3 == p1 || p3 == p2) {
			out.set(-1, -1, -1, -1);
			return out;
		}

		tmp1.sub(vertices.get(p1), vertices.get(p0));
		tmp2.sub(vertices.get(p2), vertices.get(p0));
		tmp2.cross(tmp1, tmp2);
		tmp1.sub(vertices.get(p3), vertices.get(p0));
		if (tmp1.dot(tmp2) < 0) {
			int swap_tmp = p2;
			p2 = p3;
			p3 = swap_tmp;
		}
		out.set(p0, p1, p2, p3);
		return out;
	}

	//private ConvexH convexHCrop(ConvexH convex,Plane slice);

	private void extrude(Tri t0, int v) {
		Int3 t = new Int3(t0);
		int n = triangles.size();
		Tri ta = allocateTriangle(v, t.y, t.z);
		ta.n.set(t0.n.x, n + 1, n + 2);
		triangles.get(t0.n.x).setNeighbor(t.y, t.z, n);
		Tri tb = allocateTriangle(v, t.z, t.x);
		// this t0.n.y is correct!!
		tb.n.set(t0.n.y, n + 2, n);
		triangles.get(t0.n.y).setNeighbor(t.z, t.x, n + 1);
		Tri tc = allocateTriangle(v, t.x, t.y);
		tc.n.set(t0.n.z, n, n + 1);
		triangles.get(t0.n.z).setNeighbor(t.x, t.y, n + 2);
		checkIt(ta);
		checkIt(tb);
		checkIt(tc);
		if (hasVertex(triangles.get(ta.n.x), v)) {
			removeB2B(ta, triangles.get(ta.n.x));
		}
		if (hasVertex(triangles.get(tb.n.x), v)) {
			removeB2B(tb, triangles.get(tb.n.x));
		}
		if (hasVertex(triangles.get(tc.n.x), v)) {
			removeB2B(tc, triangles.get(tc.n.x));
		}
		deAllocateTriangle(t0);
	}

	//private ConvexH test_cube();

	//BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'.
	//After the hull is generated it give you back a set of polygon faces which index the *original* point cloud.
	//The thing is, often times, there are many 'dead vertices' in the point cloud that are on longer referenced by the hull.
	//The routine 'BringOutYourDead' find only the referenced vertices, copies them to an new buffer, and re-indexes the hull so that it is a minimal representation.
	private void bringOutYourDead(ArrayList<Vector3d> verts, int vcount, ArrayList<Vector3d> overts, int[] ocount, IntArrayList indices, int indexcount) {
		IntArrayList tmpIndices = new IntArrayList();
		for (int i = 0; i < vertexIndexMapping.size(); i++) {
			tmpIndices.add(vertexIndexMapping.size());
		}

		IntArrayList usedIndices = new IntArrayList();
		MiscUtil.resize(usedIndices, vcount);
		/*
		JAVA NOTE: redudant
		for (int i=0; i<vcount; i++) {
		usedIndices.set(i, 0);
		}
		*/

		ocount[0] = 0;

		for (int i = 0; i < indexcount; i++) {
			int v = indices.get(i); // original array index

			assert (v >= 0 && v < vcount);

			if (usedIndices.get(v) != 0) { // if already remapped
				indices.set(i, usedIndices.get(v) - 1); // index to new array
			} else {
				indices.set(i, ocount[0]);      // new index mapping

				overts.get(ocount[0]).set(verts.get(v)); // copy old vert to new vert array

				for (int k = 0; k < vertexIndexMapping.size(); k++) {
					if (tmpIndices.get(k) == v) {
						vertexIndexMapping.set(k, ocount[0]);
					}
				}

				ocount[0]++; // increment output vert count

				assert (ocount[0] >= 0 && ocount[0] <= vcount);

				usedIndices.set(v, ocount[0]); // assign new index remapping
			}
		}
	}

	private static final double EPSILON = 0.000001; /* close enough to consider two btScalaring point numbers to be 'the same'. */

	private boolean cleanupVertices(int svcount,
									ArrayList<Vector3d> svertices,
									int stride,
									int[] vcount, // output number of vertices
									ArrayList<Vector3d> vertices, // location to store the results.
									double normalepsilon,
									Vector3d scale) {

		if (svcount == 0) {
			return false;
		}

		vertexIndexMapping.clear();

		vcount[0] = 0;

		double[] reciprocal = new double[3];

		if (scale != null) {
			scale.set(1, 1, 1);
		}

		double[] bmin = new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
		double[] bmax = new double[]{-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};

		ArrayList<Vector3d> vtx_ptr = svertices;
		int vtx_idx = 0;

		//	if ( 1 )
		{
			for (int i = 0; i < svcount; i++) {
				Vector3d p = vtx_ptr.get(vtx_idx);

				vtx_idx +=/*stride*/ 1;

				for (int j = 0; j < 3; j++) {
					if (VectorUtil.getCoord(p, j) < bmin[j]) {
						bmin[j] = VectorUtil.getCoord(p, j);
					}
					if (VectorUtil.getCoord(p, j) > bmax[j]) {
						bmax[j] = VectorUtil.getCoord(p, j);
					}
				}
			}
		}

		double dx = bmax[0] - bmin[0];
		double dy = bmax[1] - bmin[1];
		double dz = bmax[2] - bmin[2];

		Vector3d center = Stack.newVec();

		center.x = dx * 0.5 + bmin[0];
		center.y = dy * 0.5 + bmin[1];
		center.z = dz * 0.5 + bmin[2];

		if (dx < EPSILON || dy < EPSILON || dz < EPSILON || svcount < 3) {

			double len = Double.MAX_VALUE;

			if (dx > EPSILON && dx < len) len = dx;
			if (dy > EPSILON && dy < len) len = dy;
			if (dz > EPSILON && dz < len) len = dz;

			if (len == Double.MAX_VALUE) {
				dx = dy = dz = 0.01; // one centimeter
			} else {
				if (dx < EPSILON) dx = len * 0.05f; // 1/5th the shortest non-zero edge.
				if (dy < EPSILON) dy = len * 0.05f;
				if (dz < EPSILON) dz = len * 0.05f;
			}

			double x1 = center.x - dx;
			double x2 = center.x + dx;

			double y1 = center.y - dy;
			double y2 = center.y + dy;

			double z1 = center.z - dz;
			double z2 = center.z + dz;

			addPoint(vcount, vertices, x1, y1, z1);
			addPoint(vcount, vertices, x2, y1, z1);
			addPoint(vcount, vertices, x2, y2, z1);
			addPoint(vcount, vertices, x1, y2, z1);
			addPoint(vcount, vertices, x1, y1, z2);
			addPoint(vcount, vertices, x2, y1, z2);
			addPoint(vcount, vertices, x2, y2, z2);
			addPoint(vcount, vertices, x1, y2, z2);

			return true; // return cube
		} else {
			if (scale != null) {
				scale.x = dx;
				scale.y = dy;
				scale.z = dz;

				reciprocal[0] = 1.0 / dx;
				reciprocal[1] = 1.0 / dy;
				reciprocal[2] = 1.0 / dz;

				center.x *= reciprocal[0];
				center.y *= reciprocal[1];
				center.z *= reciprocal[2];
			}
		}

		vtx_ptr = svertices;
		vtx_idx = 0;

		for (int i = 0; i < svcount; i++) {
			Vector3d p = vtx_ptr.get(vtx_idx);
			vtx_idx +=/*stride*/ 1;

			double px = p.x;
			double py = p.y;
			double pz = p.z;

			if (scale != null) {
				px = px * reciprocal[0]; // normalize
				py = py * reciprocal[1]; // normalize
				pz = pz * reciprocal[2]; // normalize
			}

			//		if ( 1 )
			{
				int j;

				for (j = 0; j < vcount[0]; j++) {
					// XXX might be broken
					Vector3d v = vertices.get(j);

					double x = v.x;
					double y = v.y;
					double z = v.z;

					dx = Math.abs(x - px);
					dy = Math.abs(y - py);
					dz = Math.abs(z - pz);

					if (dx < normalepsilon && dy < normalepsilon && dz < normalepsilon) {
						// ok, it is close enough to the old one
						// now let us see if it is further from the center of the point cloud than the one we already recorded.
						// in which case we keep this one instead.

						double dist1 = getDist(px, py, pz, center);
						double dist2 = getDist(v.x, v.y, v.z, center);

						if (dist1 > dist2) {
							v.x = px;
							v.y = py;
							v.z = pz;
						}

						break;
					}
				}

				if (j == vcount[0]) {
					Vector3d dest = vertices.get(vcount[0]);
					dest.x = px;
					dest.y = py;
					dest.z = pz;
					vcount[0]++;
				}

				vertexIndexMapping.add(j);
			}
		}

		// ok..now make sure we didn't prune so many vertices it is now invalid.
		//	if ( 1 )
		{
			bmin = new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
			bmax = new double[]{-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};

			for (int i = 0; i < vcount[0]; i++) {
				Vector3d p = vertices.get(i);
				for (int j = 0; j < 3; j++) {
					if (VectorUtil.getCoord(p, j) < bmin[j]) {
						bmin[j] = VectorUtil.getCoord(p, j);
					}
					if (VectorUtil.getCoord(p, j) > bmax[j]) {
						bmax[j] = VectorUtil.getCoord(p, j);
					}
				}
			}

			dx = bmax[0] - bmin[0];
			dy = bmax[1] - bmin[1];
			dz = bmax[2] - bmin[2];

			if (dx < EPSILON || dy < EPSILON || dz < EPSILON || vcount[0] < 3) {
				double cx = dx * 0.5 + bmin[0];
				double cy = dy * 0.5 + bmin[1];
				double cz = dz * 0.5 + bmin[2];

				double len = Double.MAX_VALUE;

				if (dx >= EPSILON && dx < len) len = dx;
				if (dy >= EPSILON && dy < len) len = dy;
				if (dz >= EPSILON && dz < len) len = dz;

				if (len == Double.MAX_VALUE) {
					dx = dy = dz = 0.01; // one centimeter
				} else {
					if (dx < EPSILON) dx = len * 0.05f; // 1/5th the shortest non-zero edge.
					if (dy < EPSILON) dy = len * 0.05f;
					if (dz < EPSILON) dz = len * 0.05f;
				}

				double x1 = cx - dx;
				double x2 = cx + dx;

				double y1 = cy - dy;
				double y2 = cy + dy;

				double z1 = cz - dz;
				double z2 = cz + dz;

				vcount[0] = 0; // add box

				addPoint(vcount, vertices, x1, y1, z1);
				addPoint(vcount, vertices, x2, y1, z1);
				addPoint(vcount, vertices, x2, y2, z1);
				addPoint(vcount, vertices, x1, y2, z1);
				addPoint(vcount, vertices, x1, y1, z2);
				addPoint(vcount, vertices, x2, y1, z2);
				addPoint(vcount, vertices, x2, y2, z2);
				addPoint(vcount, vertices, x1, y2, z2);

				return true;
			}
		}

		return true;
	}

	////////////////////////////////////////////////////////////////////////////

	private static boolean hasVertex(Int3 t, int v) {
		return (t.getCoord(0) == v || t.getCoord(1) == v || t.getCoord(2) == v);
	}

	private static Vector3d orth(Vector3d v, Vector3d out) {
		Vector3d a = Stack.newVec();
		a.set(0.0, 0.0, 1.0);
		a.cross(v, a);

		Vector3d b = Stack.newVec();
		b.set(0.0, 1.0, 0.0);
		b.cross(v, b);

		if (a.length() > b.length()) {
			out.normalize(a);
		} else {
			out.normalize(b);
		}
		return out;
	}

	private static int maxdirfiltered(ArrayList<Vector3d> p, int count, Vector3d dir, IntArrayList allow) {
		assert (count != 0);
		int m = -1;
		for (int i = 0; i < count; i++) {
			if (allow.get(i) != 0) {
				if (m == -1 || p.get(i).dot(dir) > p.get(m).dot(dir)) {
					m = i;
				}
			}
		}
		assert (m != -1);
		return m;
	}

	private static int maxdirsterid(ArrayList<Vector3d> p, int count, Vector3d dir, IntArrayList allow) {
		Vector3d tmp = Stack.newVec();
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();
		Vector3d u = Stack.newVec();
		Vector3d v = Stack.newVec();

		int m = -1;
		while (m == -1) {
			m = maxdirfiltered(p, count, dir, allow);
			if (allow.get(m) == 3) {
				return m;
			}
			orth(dir, u);
			v.cross(u, dir);
			int ma = -1;
			for (double x = 0.0; x <= 360f; x += 45f) {
				double s = Math.sin(BulletGlobals.SIMD_RADS_PER_DEG * (x));
				double c = Math.cos(BulletGlobals.SIMD_RADS_PER_DEG * (x));

				tmp1.scale(s, u);
				tmp2.scale(c, v);
				tmp.add(tmp1, tmp2);
				tmp.scale(0.025f);
				tmp.add(dir);
				int mb = maxdirfiltered(p, count, tmp, allow);
				if (ma == m && mb == m) {
					allow.set(m, 3);
					return m;
				}
				if (ma != -1 && ma != mb) { // Yuck - this is really ugly
					int mc = ma;
					for (double xx = x - 40f; xx <= x; xx += 5f) {
						s = Math.sin(BulletGlobals.SIMD_RADS_PER_DEG * (xx));
						c = Math.cos(BulletGlobals.SIMD_RADS_PER_DEG * (xx));

						tmp1.scale(s, u);
						tmp2.scale(c, v);
						tmp.add(tmp1, tmp2);
						tmp.scale(0.025f);
						tmp.add(dir);

						int md = maxdirfiltered(p, count, tmp, allow);
						if (mc == m && md == m) {
							allow.set(m, 3);
							return m;
						}
						mc = md;
					}
				}
				ma = mb;
			}
			allow.set(m, 0);
			m = -1;
		}
		assert (false);
		return m;
	}

	private static Vector3d triNormal(Vector3d v0, Vector3d v1, Vector3d v2, Vector3d out) {
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();

		// return the normal of the triangle
		// inscribed by v0, v1, and v2
		tmp1.sub(v1, v0);
		tmp2.sub(v2, v1);
		Vector3d cp = Stack.newVec();
		cp.cross(tmp1, tmp2);
		double m = cp.lengthSquared();
		if (m == 0) {
			out.set(1.0, 0.0, 0.0);
		} else {
			out.scale(1.0 / Math.sqrt(m), cp);
		}
		Stack.subVec(3);
		return out;
	}

	private static boolean above(ArrayList<Vector3d> vertices, Int3 t, Vector3d p, double epsilon) {
		Vector3d n = triNormal(vertices.get(t.getCoord(0)), vertices.get(t.getCoord(1)), vertices.get(t.getCoord(2)), Stack.newVec());
		Vector3d tmp = Stack.borrowVec();
		tmp.sub(p, vertices.get(t.getCoord(0)));
		return n.dot(tmp) > epsilon;
	}

	private static void releaseHull(PHullResult result) {
		if (result.indices.size() != 0) {
			result.indices.clear();
		}

		result.vertexCount = 0;
		result.indexCount = 0;
		result.vertices = null;
	}

	private static void addPoint(int[] vertexCount, ArrayList<Vector3d> p, double x, double y, double z) {
		p.get(vertexCount[0]++).set(x, y, z);
	}

	private static double getDist(double px, double py, double pz, Vector3d p2) {
		double dx = px - p2.x;
		double dy = py - p2.y;
		double dz = pz - p2.z;
		return dx * dx + dy * dy + dz * dz;
	}

}
