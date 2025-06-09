// includes modifications/improvements by John Ratcliff, see BringOutYourDead below.
package com.bulletphysics.linearmath.convexhull;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * HullLibrary class can create a convex hull from a collection of vertices, using
 * the ComputeHull method. The {@link ShapeHull} class uses this HullLibrary to create
 * a approximate convex mesh given a general (non-polyhedral) convex shape.
 *
 * @author jezek2
 */
public class HullLibrary {

    public final IntArrayList vertexIndexMapping = new IntArrayList();

    private final ObjectArrayList<Tri> tris = new ObjectArrayList<>();

    /**
     * Converts point cloud to polygonal representation.
     *
     * @param desc   describes the input request
     * @param result contains the result
     * @return whether conversion was successful
     */
    public boolean createConvexHull(HullDesc desc, HullResult result) {

        PHullResult hr = new PHullResult();

        int vcount = desc.vcount;
        if (vcount < 8) vcount = 8;

        ObjectArrayList<Vector3d> vertexSource = new ObjectArrayList<>();
        MiscUtil.resize(vertexSource, vcount, Vector3d.class);

        Vector3d scale = Stack.newVec();

        int[] ovcount = new int[1];

        boolean ok = cleanupVertices(desc.vcount, desc.vertices, ovcount, vertexSource, desc.normalEpsilon, scale); // normalize point cloud, remove duplicates!
        if (!ok) {
            Stack.subVec(1);
            return false;
        }

        // scale vertices back to their original size.
        for (int i = 0; i < ovcount[0]; i++) {
            Vector3d v = vertexSource.getQuick(i);
            VectorUtil.mul(v, v, scale);
        }

        ok = computeHull(ovcount[0], vertexSource, hr, desc.maxVertices);
        if (!ok) {
            Stack.subVec(1);
            return false;
        }

        // re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
        ObjectArrayList<Vector3d> vertexScratch = new ObjectArrayList<>();
        MiscUtil.resize(vertexScratch, hr.vertexCount, Vector3d.class);

        bringOutYourDead(hr.vertices, hr.vertexCount, vertexScratch, ovcount, hr.indices, hr.indexCount);

        if (desc.hasHullFlag(HullFlags.TRIANGLES)) { // if he wants the results as triangle!
            result.polygons = false;
            result.numOutputVertices = ovcount[0];
            MiscUtil.resize(result.outputVertices, ovcount[0], Vector3d.class);
            result.numFaces = hr.faceCount;
            result.numIndices = hr.indexCount;

            MiscUtil.resize(result.indices, hr.indexCount, 0);

            for (int i = 0; i < ovcount[0]; i++) {
                result.outputVertices.getQuick(i).set(vertexScratch.getQuick(i));
            }

            if (desc.hasHullFlag(HullFlags.REVERSE_ORDER)) {
                IntArrayList source_ptr = hr.indices;
                int source_idx = 0;

                IntArrayList dest_ptr = result.indices;
                int dest_idx = 0;

                for (int i = 0; i < hr.faceCount; i++) {
                    dest_ptr.set(dest_idx, source_ptr.get(source_idx + 2));
                    dest_ptr.set(dest_idx + 1, source_ptr.get(source_idx + 1));
                    dest_ptr.set(dest_idx + 2, source_ptr.get(source_idx));
                    dest_idx += 3;
                    source_idx += 3;
                }
            } else {
                for (int i = 0; i < hr.indexCount; i++) {
                    result.indices.set(i, hr.indices.get(i));
                }
            }
        } else {
            result.polygons = true;
            result.numOutputVertices = ovcount[0];
            MiscUtil.resize(result.outputVertices, ovcount[0], Vector3d.class);
            result.numFaces = hr.faceCount;
            result.numIndices = hr.indexCount + hr.faceCount;
            MiscUtil.resize(result.indices, result.numIndices, 0);
            for (int i = 0; i < ovcount[0]; i++) {
                result.outputVertices.getQuick(i).set(vertexScratch.getQuick(i));
            }

            IntArrayList source_ptr = hr.indices;
            int source_idx = 0;

            IntArrayList dest_ptr = result.indices;
            int dest_idx = 0;

            for (int i = 0; i < hr.faceCount; i++) {
                dest_ptr.set(dest_idx, 3);
                if (desc.hasHullFlag(HullFlags.REVERSE_ORDER)) {
                    dest_ptr.set(dest_idx + 1, source_ptr.get(source_idx + 2));
                    dest_ptr.set(dest_idx + 2, source_ptr.get(source_idx + 1));
                    dest_ptr.set(dest_idx + 3, source_ptr.get(source_idx));
                } else {
                    dest_ptr.set(dest_idx + 1, source_ptr.get(source_idx));
                    dest_ptr.set(dest_idx + 2, source_ptr.get(source_idx + 1));
                    dest_ptr.set(dest_idx + 3, source_ptr.get(source_idx + 2));
                }

                dest_idx += 4;
                source_idx += 3;
            }
        }
        releaseHull(hr);

        Stack.subVec(1);
        return true;
    }

    /**
     * Release memory allocated for this result, we are done with it.
     */
    public void releaseResult(HullResult result) {
        if (!result.outputVertices.isEmpty()) {
            result.numOutputVertices = 0;
            result.outputVertices.clear();
        }
        if (result.indices.size() != 0) {
            result.numIndices = 0;
            result.indices.clear();
        }
    }

    private boolean computeHull(int vcount, ObjectArrayList<Vector3d> vertices, PHullResult result, int vlimit) {
        int[] tris_count = new int[1];
        int ret = calcHull(vertices, vcount, result.indices, tris_count, vlimit);
        if (ret == 0) return false;
        result.indexCount = tris_count[0] * 3;
        result.faceCount = tris_count[0];
        result.vertices = vertices;
        result.vertexCount = vcount;
        return true;
    }

    private Tri allocateTriangle(int a, int b, int c) {
        Tri tr = new Tri(a, b, c);
        tr.id = tris.size();
        tris.add(tr);

        return tr;
    }

    private void deAllocateTriangle(Tri tri) {
        assert (tris.getQuick(tri.id) == tri);
        tris.setQuick(tri.id, null);
    }

    private void b2bfix(Tri s, Tri t) {
        for (int i = 0; i < 3; i++) {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            int a = s.getCoord(i1);
            int b = s.getCoord(i2);
            assert (tris.getQuick(s.neibGet(a, b)).neibGet(b, a) == s.id);
            assert (tris.getQuick(t.neibGet(a, b)).neibGet(b, a) == t.id);
            tris.getQuick(s.neibGet(a, b)).neibSet(b, a, t.neibGet(b, a));
            tris.getQuick(t.neibGet(b, a)).neibSet(a, b, s.neibGet(a, b));
        }
    }

    private void removeb2b(Tri s, Tri t) {
        b2bfix(s, t);
        deAllocateTriangle(s);

        deAllocateTriangle(t);
    }

    private void checkit(Tri t) {
        assert (tris.getQuick(t.id) == t);
        for (int i = 0; i < 3; i++) {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            int a = t.getCoord(i1);
            int b = t.getCoord(i2);

            assert (a != b);
            assert (tris.getQuick(t.n.getCoord(i)).neibGet(b, a) == t.id);
        }
    }

    private Tri extrudable(double epsilon) {
        Tri t = null;
        for (int i = 0; i < tris.size(); i++) {
            if (t == null || (tris.getQuick(i) != null && t.rise < tris.getQuick(i).rise)) {
                t = tris.getQuick(i);
            }
        }
        return t != null && (t.rise > epsilon) ? t : null;
    }

    private int calcHull(
            ObjectArrayList<Vector3d> vertices, int vertexCount, IntArrayList trisOut,
            int[] trisCount, int vertexLimit) {
        int rc = calcHullGen(vertices, vertexCount, vertexLimit);
        if (rc == 0) return 0;

        IntArrayList ts = new IntArrayList();

        for (int i = 0; i < tris.size(); i++) {
            if (tris.getQuick(i) != null) {
                for (int j = 0; j < 3; j++) {
                    ts.add((tris.getQuick(i)).getCoord(j));
                }
                deAllocateTriangle(tris.getQuick(i));
            }
        }
        trisCount[0] = ts.size() / 3;
        MiscUtil.resize(trisOut, ts.size(), 0);

        for (int i = 0; i < ts.size(); i++) {
            trisOut.set(i, ts.get(i));
        }
        MiscUtil.resize(tris, 0, Tri.class);

        return 1;
    }

    private int calcHullGen(ObjectArrayList<Vector3d> verts, int verts_count, int vlimit) {
        if (verts_count < 4) return 0;

        Vector3d tmp = Stack.newVec();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        if (vlimit == 0) {
            vlimit = 1000000000;
        }
        //int j;
        Vector3d bmin = Stack.newVec(verts.getQuick(0));
        Vector3d bmax = Stack.newVec(verts.getQuick(0));
        IntArrayList isextreme = new IntArrayList();
        //isextreme.reserve(verts_count);
        IntArrayList allow = new IntArrayList();
        //allow.reserve(verts_count);

        for (int j = 0; j < verts_count; j++) {
            allow.add(1);
            isextreme.add(0);
            VectorUtil.setMin(bmin, verts.getQuick(j));
            VectorUtil.setMax(bmax, verts.getQuick(j));
        }
        tmp.sub(bmax, bmin);
        double epsilon = tmp.length() * 0.001f;
        assert (epsilon != 0.0);

        Int4 p = findSimplex(verts, verts_count, allow, new Int4());
        if (p.x == -1) {
            Stack.subVec(5);
            return 0; // simplex failed
            // a valid interior point
        }

        Vector3d center = Stack.newVec();
        VectorUtil.add(center, verts.getQuick(p.x), verts.getQuick(p.y), verts.getQuick(p.z), verts.getQuick(p.w));
        center.scale(1.0 / 4f);

        Tri t0 = allocateTriangle(p.z, p.w, p.y);
        t0.n.set(2, 3, 1);
        Tri t1 = allocateTriangle(p.w, p.z, p.x);
        t1.n.set(3, 2, 0);
        Tri t2 = allocateTriangle(p.x, p.y, p.w);
        t2.n.set(0, 1, 3);
        Tri t3 = allocateTriangle(p.y, p.x, p.z);
        t3.n.set(1, 0, 2);
        isextreme.set(p.x, 1);
        isextreme.set(p.y, 1);
        isextreme.set(p.z, 1);
        isextreme.set(p.w, 1);
        checkit(t0);
        checkit(t1);
        checkit(t2);
        checkit(t3);

        Vector3d n = Stack.newVec();

        for (int j = 0; j < tris.size(); j++) {
            Tri t = tris.getQuick(j);
            assert (t != null);
            assert (t.maxValue < 0);
            triNormal(verts.getQuick(t.x), verts.getQuick(t.y), verts.getQuick(t.z), n);
            t.maxValue = maxdirsterid(verts, verts_count, n, allow);
            tmp.sub(verts.getQuick(t.maxValue), verts.getQuick(t.x));
            t.rise = n.dot(tmp);
        }
        Tri te;
        vlimit -= 4;
        while (vlimit > 0 && ((te = extrudable(epsilon)) != null)) {
            int v = te.maxValue;
            assert (v != -1);
            assert (isextreme.get(v) == 0);  // wtf we've already done this vertex
            isextreme.set(v, 1);
            //if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
            int j = tris.size();
            while ((j--) != 0) {
                if (tris.getQuick(j) == null) {
                    continue;
                }
                Int3 t = tris.getQuick(j);
                if (above(verts, t, verts.getQuick(v), 0.01f * epsilon)) {
                    extrude(tris.getQuick(j), v);
                }
            }
            // now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
            j = tris.size();
            while ((j--) != 0) {
                if (tris.getQuick(j) == null) {
                    continue;
                }
                if (!hasvert(tris.getQuick(j), v)) {
                    break;
                }
                Int3 nt = tris.getQuick(j);
                tmp1.sub(verts.getQuick(nt.y), verts.getQuick(nt.x));
                tmp2.sub(verts.getQuick(nt.z), verts.getQuick(nt.y));
                tmp.cross(tmp1, tmp2);
                if (above(verts, nt, center, 0.01f * epsilon) || tmp.length() < epsilon * epsilon * 0.1) {
                    Tri nb = tris.getQuick(tris.getQuick(j).n.x);
                    assert (nb != null);
                    assert (!hasvert(nb, v));
                    assert (nb.id < j);
                    extrude(nb, v);
                    j = tris.size();
                }
            }
            j = tris.size();
            while ((j--) != 0) {
                Tri t = tris.getQuick(j);
                if (t == null) {
                    continue;
                }
                if (t.maxValue >= 0) {
                    break;
                }
                triNormal(verts.getQuick(t.x), verts.getQuick(t.y), verts.getQuick(t.z), n);
                t.maxValue = maxdirsterid(verts, verts_count, n, allow);
                if (isextreme.get(t.maxValue) != 0) {
                    t.maxValue = -1; // already done that vertex - algorithm needs to be able to terminate.
                } else {
                    tmp.sub(verts.getQuick(t.maxValue), verts.getQuick(t.x));
                    t.rise = n.dot(tmp);
                }
            }
            vlimit--;
        }
        Stack.subVec(7);
        return 1;
    }

    private Int4 findSimplex(ObjectArrayList<Vector3d> verts, int verts_count, IntArrayList allow, Int4 out) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        Vector3d basisX = Stack.newVec();
        Vector3d basisY = Stack.newVec();
        Vector3d basisZ = Stack.newVec();
        basisX.set(0.01f, 0.02f, 1.0);
        int p0 = maxdirsterid(verts, verts_count, basisX, allow);
        tmp.negate(basisX);
        int p1 = maxdirsterid(verts, verts_count, tmp, allow);
        basisX.sub(verts.getQuick(p0), verts.getQuick(p1));
        if (p0 == p1 || (basisX.x == 0.0 && basisX.y == 0.0 && basisX.z == 0.0)) {
            out.set(-1, -1, -1, -1);
            Stack.subVec(6);
            return out;
        }
        tmp.set(1.0, 0.02f, 0.0);
        basisY.cross(tmp, basisX);
        tmp.set(-0.02f, 1.0, 0.0);
        basisZ.cross(tmp, basisX);
        if (basisY.length() > basisZ.length()) {
            basisY.normalize();
        } else {
            basisY.set(basisZ);
            basisY.normalize();
        }
        int p2 = maxdirsterid(verts, verts_count, basisY, allow);
        if (p2 == p0 || p2 == p1) {
            tmp.negate(basisY);
            p2 = maxdirsterid(verts, verts_count, tmp, allow);
        }
        if (p2 == p0 || p2 == p1) {
            out.set(-1, -1, -1, -1);
            Stack.subVec(6);
            return out;
        }
        basisY.sub(verts.getQuick(p2), verts.getQuick(p0));
        basisZ.cross(basisY, basisX);
        basisZ.normalize();
        int p3 = maxdirsterid(verts, verts_count, basisZ, allow);
        if (p3 == p0 || p3 == p1 || p3 == p2) {
            tmp.negate(basisZ);
            p3 = maxdirsterid(verts, verts_count, tmp, allow);
        }
        if (p3 == p0 || p3 == p1 || p3 == p2) {
            out.set(-1, -1, -1, -1);
            Stack.subVec(6);
            return out;
        }

        tmp1.sub(verts.getQuick(p1), verts.getQuick(p0));
        tmp2.sub(verts.getQuick(p2), verts.getQuick(p0));
        tmp2.cross(tmp1, tmp2);
        tmp1.sub(verts.getQuick(p3), verts.getQuick(p0));
        if (tmp1.dot(tmp2) < 0) {
            int swap_tmp = p2;
            p2 = p3;
            p3 = swap_tmp;
        }
        out.set(p0, p1, p2, p3);
        Stack.subVec(6);
        return out;
    }

    //private ConvexH convexHCrop(ConvexH convex,Plane slice);

    private void extrude(Tri t0, int v) {
        Int3 t = new Int3(t0);
        int n = tris.size();
        Tri ta = allocateTriangle(v, t.y, t.z);
        ta.n.set(t0.n.x, n + 1, n + 2);
        tris.getQuick(t0.n.x).neibSet(t.y, t.z, n);
        Tri tb = allocateTriangle(v, t.z, t.x);
        //noinspection SuspiciousNameCombination
        tb.n.set(t0.n.y, n + 2, n);
        tris.getQuick(t0.n.y).neibSet(t.z, t.x, n + 1);
        Tri tc = allocateTriangle(v, t.x, t.y);
        tc.n.set(t0.n.z, n, n + 1);
        tris.getQuick(t0.n.z).neibSet(t.x, t.y, n + 2);
        checkit(ta);
        checkit(tb);
        checkit(tc);
        if (hasvert(tris.getQuick(ta.n.x), v)) {
            removeb2b(ta, tris.getQuick(ta.n.x));
        }
        if (hasvert(tris.getQuick(tb.n.x), v)) {
            removeb2b(tb, tris.getQuick(tb.n.x));
        }
        if (hasvert(tris.getQuick(tc.n.x), v)) {
            removeb2b(tc, tris.getQuick(tc.n.x));
        }
        deAllocateTriangle(t0);
    }

    //private ConvexH test_cube();

    /**
     * BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'.
     * After the hull is generated it give you back a set of polygon faces which index the *original* point cloud.
     * The thing is, often times, there are many 'dead vertices' in the point cloud that are on longer referenced by the hull.
     * The routine 'BringOutYourDead' find only the referenced vertices, copies them to an new buffer, and re-indexes the hull so that it is a minimal representation.
     **/
    private void bringOutYourDead(ObjectArrayList<Vector3d> verts, int vcount, ObjectArrayList<Vector3d> overts, int[] ocount, IntArrayList indices, int indexcount) {
        IntArrayList tmpIndices = new IntArrayList();
        for (int i = 0; i < vertexIndexMapping.size(); i++) {
            tmpIndices.add(vertexIndexMapping.size());
        }

        IntArrayList usedIndices = new IntArrayList();
        MiscUtil.resize(usedIndices, vcount, 0);
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

                overts.getQuick(ocount[0]).set(verts.getQuick(v)); // copy old vert to new vert array

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

    private static final double EPSILON = 0.000001f; /* close enough to consider two btScalaring point numbers to be 'the same'. */

    private boolean cleanupVertices(
            int svcount,
            ObjectArrayList<Vector3d> svertices,
            int[] vcount, // output number of vertices
            ObjectArrayList<Vector3d> vertices, // location to store the results.
            double normalepsilon,
            Vector3d scale) {

        if (svcount == 0) {
            return false;
        }

        vertexIndexMapping.clear();

        vcount[0] = 0;

        double[] recip = new double[3];

        if (scale != null) {
            scale.set(1, 1, 1);
        }

        Vector3d bmin = new Vector3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        Vector3d bmax = new Vector3d(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);

        ObjectArrayList<Vector3d> vtxPtr = svertices;
        int vtxIdx = 0;

        for (int i = 0; i < svcount; i++) {
            Vector3d p = vtxPtr.getQuick(vtxIdx);

            vtxIdx++;

            VectorUtil.setMin(bmin, p);
            VectorUtil.setMax(bmax, p);
        }

        double dx = bmax.x - bmin.x;
        double dy = bmax.y - bmin.y;
        double dz = bmax.z - bmin.z;

        Vector3d center = Stack.newVec();

        center.x = dx * 0.5 + bmin.x;
        center.y = dy * 0.5 + bmin.y;
        center.z = dz * 0.5 + bmin.z;

        if (dx < EPSILON || dy < EPSILON || dz < EPSILON || svcount < 3) {

            double len = Float.MAX_VALUE;

            if (dx > EPSILON && dx < len) len = dx;
            if (dy > EPSILON && dy < len) len = dy;
            if (dz > EPSILON && dz < len) len = dz;

            if (len == Float.MAX_VALUE) {
                dx = dy = dz = 0.01f; // one centimeter
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

                recip[0] = 1.0 / dx;
                recip[1] = 1.0 / dy;
                recip[2] = 1.0 / dz;

                center.x *= recip[0];
                center.y *= recip[1];
                center.z *= recip[2];
            }
        }

        vtxPtr = svertices;
        vtxIdx = 0;

        for (int i = 0; i < svcount; i++) {
            Vector3d p = vtxPtr.getQuick(vtxIdx);
            vtxIdx +=/*stride*/ 1;

            double px = p.x;
            double py = p.y;
            double pz = p.z;

            if (scale != null) {
                px = px * recip[0]; // normalize
                py = py * recip[1]; // normalize
                pz = pz * recip[2]; // normalize
            }

            //		if ( 1 )
            {
                int j;

                for (j = 0; j < vcount[0]; j++) {
                    // XXX might be broken
                    Vector3d v = vertices.getQuick(j);

                    dx = Math.abs(v.x - px);
                    dy = Math.abs(v.y - py);
                    dz = Math.abs(v.z - pz);

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
                    vertices.getQuick(vcount[0]).set(px, py, pz);
                    vcount[0]++;
                }

                vertexIndexMapping.add(j);
            }
        }

        // ok..now make sure we didn't prune so many vertices it is now invalid.
        //	if ( 1 )
        {
            bmin.set(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            bmax.set(-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE);

            for (int i = 0; i < vcount[0]; i++) {
                Vector3d p = vertices.getQuick(i);
                VectorUtil.setMin(bmin, p);
                VectorUtil.setMax(bmax, p);
            }

            dx = bmax.x - bmin.x;
            dy = bmax.y - bmin.y;
            dz = bmax.z - bmin.z;

            if (dx < EPSILON || dy < EPSILON || dz < EPSILON || vcount[0] < 3) {
                double cx = dx * 0.5 + bmin.x;
                double cy = dy * 0.5 + bmin.y;
                double cz = dz * 0.5 + bmin.z;

                double len = Float.MAX_VALUE;

                if (dx >= EPSILON && dx < len) len = dx;
                if (dy >= EPSILON && dy < len) len = dy;
                if (dz >= EPSILON && dz < len) len = dz;

                if (len == Float.MAX_VALUE) {
                    dx = dy = dz = 0.01f; // one centimeter
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

    /// /////////////////////////////////////////////////////////////////////////

    private static boolean hasvert(Int3 t, int v) {
        return (t.x == v || t.y == v || t.z == v);
    }

    private static void findOrthogonalVector(Vector3d v, Vector3d out) {
        Vector3d vCrossZ = Stack.newVec();
        vCrossZ.set(0.0, 0.0, 1.0);
        vCrossZ.cross(v, vCrossZ);

        Vector3d vCrossY = Stack.newVec();
        vCrossY.set(0.0, 1.0, 0.0);
        vCrossY.cross(v, vCrossY);

        if (vCrossZ.length() > vCrossY.length()) {
            out.normalize(vCrossZ);
        } else {
            out.normalize(vCrossY);
        }
        Stack.subVec(2);
    }

    private static int maxdirfiltered(ObjectArrayList<Vector3d> p, int count, Vector3d dir, IntArrayList allow) {
        assert (count != 0);
        int m = -1;
        for (int i = 0; i < count; i++) {
            if (allow.get(i) != 0) {
                if (m == -1 || p.getQuick(i).dot(dir) > p.getQuick(m).dot(dir)) {
                    m = i;
                }
            }
        }
        assert (m != -1);
        return m;
    }

    private static int maxdirsterid(ObjectArrayList<Vector3d> p, int count, Vector3d dir, IntArrayList allow) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();
        Vector3d u = Stack.newVec();
        Vector3d v = Stack.newVec();

        while (true) {
            int m = maxdirfiltered(p, count, dir, allow);
            if (allow.get(m) == 3) {
                Stack.subVec(5);
                return m;
            }
            findOrthogonalVector(dir, u);
            v.cross(u, dir);
            int ma = -1;
            for (double x = 0.0; x <= 360.0; x += 45.0) {
                double s = Math.sin(BulletGlobals.SIMD_RADS_PER_DEG * (x));
                double c = Math.cos(BulletGlobals.SIMD_RADS_PER_DEG * (x));

                tmp1.scale(s, u);
                tmp2.scale(c, v);
                tmp.add(tmp1, tmp2);
                tmp.scale(0.025);
                tmp.add(dir);
                int mb = maxdirfiltered(p, count, tmp, allow);
                if (ma == m && mb == m) {
                    allow.set(m, 3);
                    Stack.subVec(5);
                    return m;
                }
                if (ma != -1 && ma != mb) { // Yuck - this is really ugly
                    int mc = ma;
                    for (double xx = x - 40.0; xx <= x; xx += 5.0) {
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
                            Stack.subVec(5);
                            return m;
                        }
                        mc = md;
                    }
                }
                ma = mb;
            }
            allow.set(m, 0);
        }
    }

    private static Vector3d triNormal(Vector3d v0, Vector3d v1, Vector3d v2, Vector3d out) {
        Vector3d cross = Stack.borrowVec();

        // return the normal of the triangle
        // inscribed by v0, v1, and v2
        cross.sub(v1, v0);
        out.sub(v2, v1);
        cross.cross(cross, out);
        double m = cross.length();
        if (m == 0) {
            out.set(1.0, 0.0, 0.0);
            return out;
        }
        out.scale(1.0 / m, cross);
        return out;
    }

    private static boolean above(ObjectArrayList<Vector3d> vertices, Int3 t, Vector3d p, double epsilon) {
        Vector3d n = triNormal(vertices.getQuick(t.x), vertices.getQuick(t.y), vertices.getQuick(t.z), Stack.newVec());
        Vector3d tmp = Stack.newVec();
        tmp.sub(p, vertices.getQuick(t.x));
        Stack.subVec(2);
        return (n.dot(tmp) > epsilon); // EPSILON???
    }

    private static void releaseHull(PHullResult result) {
        if (result.indices.size() != 0) {
            result.indices.clear();
        }

        result.vertexCount = 0;
        result.indexCount = 0;
        result.vertices = null;
    }

    private static void addPoint(int[] vertexCount, ObjectArrayList<Vector3d> p, double x, double y, double z) {
        // XXX, might be broken
        p.getQuick(vertexCount[0]).set(x, y, z);
        vertexCount[0]++;
    }

    private static double getDist(double px, double py, double pz, Vector3d p2) {
        double dx = px - p2.x;
        double dy = py - p2.y;
        double dz = pz - p2.z;
        return dx * dx + dy * dy + dz * dz;
    }

}
