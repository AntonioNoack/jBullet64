/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectStackList;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.Arrays;

/*
GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

/**
 * GjkEpaSolver contributed under zlib by Nathanael Presson.
 *
 * @author jezek2
 */
public class GjkEpaSolver {

    protected final ArrayPool<double[]> doubleArrays = ArrayPool.get(double.class);

    protected final ObjectStackList<Mkv> stackMkv = new ObjectStackList<>(Mkv.class);
    protected final ObjectStackList<He> stackHe = new ObjectStackList<>(He.class);
    protected final ObjectStackList<Face> stackFace = new ObjectStackList<>(Face.class);

    protected void pushStack() {
        stackMkv.push();
        stackHe.push();
        stackFace.push();
    }

    protected void popStack() {
        stackMkv.pop();
        stackHe.pop();
        stackFace.pop();
    }

    public enum ResultsStatus {
        Separated,        /* Shapes don't penetrate												*/
        Penetrating,    /* Shapes are penetrating												*/
        GJK_Failed,        /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
        EPA_Failed,        /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
    }

    public static class Results {
        public ResultsStatus status;
        public final Vector3d[] witnesses = new Vector3d[]{new Vector3d(), new Vector3d()};
        public final Vector3d normal = new Vector3d();
        public double depth;
        public int epa_iterations;
        public int gjk_iterations;
    }

    ////////////////////////////////////////////////////////////////////////////

    private static final double cstInf = BulletGlobals.SIMD_INFINITY;
    private static final double cstPi = BulletGlobals.SIMD_PI;
    private static final double cst2Pi = BulletGlobals.SIMD_2_PI;
    private static final int GJK_maxiterations = 128;
    private static final int GJK_hashsize = 1 << 6;
    private static final int GJK_hashmask = GJK_hashsize - 1;
    private static final double GJK_insimplex_eps = 0.0001;
    private static final double GJK_sqinsimplex_eps = GJK_insimplex_eps * GJK_insimplex_eps;
    private static final int EPA_maxiterations = 256;
    private static final double EPA_inface_eps = 0.01;
    private static final double EPA_accuracy = 0.001;

    ////////////////////////////////////////////////////////////////////////////

    public static class Mkv {
        public final Vector3d w = new Vector3d(); // Minkowski vertex
        public final Vector3d r = new Vector3d(); // Ray

        public void set(Mkv m) {
            w.set(m.w);
            r.set(m.r);
        }
    }

    public static class He {
        public final Vector3d v = new Vector3d();
        public He n;
    }

    protected class GJK {
        //protected final BulletStack stack = BulletStack.get();

        //public btStackAlloc sa;
        //public Block sablock;
        public final He[] table = new He[GJK_hashsize];
        public final Matrix3d[] wrotations/*[2]*/ = new Matrix3d[]{new Matrix3d(), new Matrix3d()};
        public final Vector3d[] positions/*[2]*/ = new Vector3d[]{new Vector3d(), new Vector3d()};
        public final ConvexShape[] shapes = new ConvexShape[2];
        public final Mkv[] simplex = new Mkv[5];
        public final Vector3d ray = new Vector3d();
        public /*unsigned*/ int order;
        public /*unsigned*/ int iterations;
        public double margin;
        public boolean failed;

        {
            for (int i = 0; i < simplex.length; i++) simplex[i] = new Mkv();
        }

        public GJK() {
        }

        public GJK(/*StackAlloc psa,*/
                Matrix3d wrot0, Vector3d pos0, ConvexShape shape0,
                Matrix3d wrot1, Vector3d pos1, ConvexShape shape1) {
            this(wrot0, pos0, shape0, wrot1, pos1, shape1, 0f);
        }

        public GJK(/*StackAlloc psa,*/
                Matrix3d wrot0, Vector3d pos0, ConvexShape shape0,
                Matrix3d wrot1, Vector3d pos1, ConvexShape shape1,
                double pmargin) {
            init(wrot0, pos0, shape0, wrot1, pos1, shape1, pmargin);
        }

        public void init(/*StackAlloc psa,*/
                Matrix3d wrot0, Vector3d pos0, ConvexShape shape0,
                Matrix3d wrot1, Vector3d pos1, ConvexShape shape1,
                double pmargin) {
            pushStack();
            wrotations[0].set(wrot0);
            positions[0].set(pos0);
            shapes[0] = shape0;
            wrotations[1].set(wrot1);
            positions[1].set(pos1);
            shapes[1] = shape1;
            //sa		=psa;
            //sablock	=sa->beginBlock();
            margin = pmargin;
            failed = false;
        }

        public void destroy() {
            popStack();
        }

        // vdh: very dummy hash
        public /*unsigned*/ int Hash(Vector3d v) {
            int h = (int) (v.x * 15461) ^ (int) (v.y * 83003) ^ (int) (v.z * 15473);
            return (h * 169639) & GJK_hashmask;
        }

        public Vector3d LocalSupport(Vector3d d, /*unsigned*/ int i, Vector3d out) {
            Vector3d tmp = Stack.newVec();
            MatrixUtil.transposeTransform(tmp, d, wrotations[i]);

            shapes[i].localGetSupportingVertex(tmp, out);
            wrotations[i].transform(out);
            out.add(positions[i]);

            Stack.subVec(1);

            return out;
        }

        public void Support(Vector3d d, Mkv v) {
            v.r.set(d);

            Vector3d tmp1 = LocalSupport(d, 0, Stack.newVec());

            Vector3d tmp = Stack.newVec();
            tmp.set(d);
            tmp.negate();
            Vector3d tmp2 = LocalSupport(tmp, 1, Stack.newVec());

            v.w.sub(tmp1, tmp2);
            v.w.scaleAdd(margin, d, v.w);

            Stack.subVec(3);

        }

        public boolean FetchSupport() {
            int h = Hash(ray);
            He e = table[h];
            while (e != null) {
                if (e.v.equals(ray)) {
                    --order;
                    return false;
                } else {
                    e = e.n;
                }
            }
            //e = (He*)sa->allocate(sizeof(He));
            //e = new He();
            e = stackHe.get();
            e.v.set(ray);
            e.n = table[h];
            table[h] = e;
            Support(ray, simplex[++order]);
            return (ray.dot(simplex[order].w) > 0);
        }

        public boolean SolveSimplex2(Vector3d ao, Vector3d ab) {
            if (ab.dot(ao) >= 0) {
                Vector3d cabo = Stack.borrowVec();
                cabo.cross(ab, ao);
                if (cabo.lengthSquared() > GJK_sqinsimplex_eps) {
                    ray.cross(cabo, ab);
                } else {
                    return true;
                }
            } else {
                order = 0;
                simplex[0].set(simplex[1]);
                ray.set(ao);
            }
            return (false);
        }

        /*private double crossLengthSquared(Vector3d a, Vector3d b) {
            double x = a.y * b.z - a.z * b.y;
            double y = b.x * a.z - b.z * a.x;
            double z = a.x * b.y - a.y * b.x;
            return x * x + y * y + z * z;
        }*/

        public boolean SolveSimplex3(Vector3d ao, Vector3d ab, Vector3d ac) {
            Vector3d tmp = Stack.newVec();
            tmp.cross(ab, ac);
            boolean result = SolveSimplex3a(ao, ab, ac, tmp);
            Stack.subVec(1);
            return result;
        }

        public boolean SolveSimplex3a(Vector3d ao, Vector3d ab, Vector3d ac, Vector3d cabc) {
            // TODO: optimize

            Vector3d tmp = Stack.newVec();
            tmp.cross(cabc, ab);

            Vector3d tmp2 = Stack.newVec();
            tmp2.cross(cabc, ac);

            // we won't use new vectors, as long, as we need them
            Stack.subVec(2);

            if (tmp.dot(ao) < -GJK_insimplex_eps) {
                order = 1;
                simplex[0].set(simplex[1]);
                simplex[1].set(simplex[2]);
                return SolveSimplex2(ao, ab);
            } else if (tmp2.dot(ao) > +GJK_insimplex_eps) {
                order = 1;
                simplex[1].set(simplex[2]);
                return SolveSimplex2(ao, ac);
            } else {
                double d = cabc.dot(ao);
                if (Math.abs(d) > GJK_insimplex_eps) {
                    if (d > 0) {
                        ray.set(cabc);
                    } else {
                        ray.negate(cabc);

                        Mkv swapTmp = new Mkv();
                        swapTmp.set(simplex[0]);
                        simplex[0].set(simplex[1]);
                        simplex[1].set(swapTmp);
                    }
                    return false;
                } else {
                    return true;
                }
            }
        }

        public boolean SolveSimplex4(Vector3d ao, Vector3d ab, Vector3d ac, Vector3d ad) {
            // TODO: optimize

            Vector3d crs = Stack.newVec();

            Vector3d tmp = Stack.newVec();
            tmp.cross(ab, ac);

            Vector3d tmp2 = Stack.newVec();
            tmp2.cross(ac, ad);

            Vector3d tmp3 = Stack.newVec();
            tmp3.cross(ad, ab);

            boolean answer = true;
            if (tmp.dot(ao) > GJK_insimplex_eps) {
                crs.set(tmp);
                order = 2;
                simplex[0].set(simplex[1]);
                simplex[1].set(simplex[2]);
                simplex[2].set(simplex[3]);
                answer = SolveSimplex3a(ao, ab, ac, crs);
            } else if (tmp2.dot(ao) > GJK_insimplex_eps) {
                crs.set(tmp2);
                order = 2;
                simplex[2].set(simplex[3]);
                answer = SolveSimplex3a(ao, ac, ad, crs);
            } else if (tmp3.dot(ao) > GJK_insimplex_eps) {
                crs.set(tmp3);
                order = 2;
                simplex[1].set(simplex[0]);
                simplex[0].set(simplex[2]);
                simplex[2].set(simplex[3]);
                answer = SolveSimplex3a(ao, ad, ab, crs);
            }
            Stack.subVec(4);
            return answer;
        }

        public boolean SearchOrigin() {
            Vector3d tmp = Stack.newVec();
            tmp.set(1.0, 0.0, 0.0);
            boolean result = SearchOrigin(tmp);
            Stack.subVec(1);
            return result;
        }

        public boolean SearchOrigin(Vector3d initray) {

            Vector3d tmp1 = Stack.newVec();
            Vector3d tmp2 = Stack.newVec();
            Vector3d tmp3 = Stack.newVec();
            Vector3d tmp4 = Stack.newVec();

            iterations = 0;
            order = -1;
            failed = false;
            ray.set(initray);
            ray.normalize();

            Arrays.fill(table, null);

            FetchSupport();
            ray.negate(simplex[0].w);
            for (; iterations < GJK_maxiterations; ++iterations) {
                double rl = ray.length();
                ray.scale(1.0 / (rl > 0f ? rl : 1f));
                if (FetchSupport()) {
                    boolean found = false;
                    switch (order) {
                        case 1: {
                            tmp1.negate(simplex[1].w);
                            tmp2.sub(simplex[0].w, simplex[1].w);
                            found = SolveSimplex2(tmp1, tmp2);
                            break;
                        }
                        case 2: {
                            tmp1.negate(simplex[2].w);
                            tmp2.sub(simplex[1].w, simplex[2].w);
                            tmp3.sub(simplex[0].w, simplex[2].w);
                            found = SolveSimplex3(tmp1, tmp2, tmp3);
                            break;
                        }
                        case 3: {
                            tmp1.negate(simplex[3].w);
                            tmp2.sub(simplex[2].w, simplex[3].w);
                            tmp3.sub(simplex[1].w, simplex[3].w);
                            tmp4.sub(simplex[0].w, simplex[3].w);
                            found = SolveSimplex4(tmp1, tmp2, tmp3, tmp4);
                            break;
                        }
                    }
                    if (found) {
                        Stack.subVec(4);
                        return true;
                    }
                } else {
                    Stack.subVec(4);
                    return false;
                }
            }
            failed = true;
            Stack.subVec(4);
            return false;
        }

        public boolean EncloseOrigin() {

            switch (order) {
                // Point
                case 0:
                    break;
                // Line
                case 1: {
                    Vector3d tmp = Stack.newVec();
                    Vector3d ab = Stack.newVec();
                    ab.sub(simplex[1].w, simplex[0].w);

                    Vector3d b0 = Stack.newVec(), b1 = Stack.newVec(), b2 = Stack.newVec();
                    b0.set(1.0, 0.0, 0.0);
                    b1.set(0.0, 1.0, 0.0);
                    b2.set(0.0, 0.0, 1.0);

                    b0.cross(ab, b0);
                    b1.cross(ab, b1);
                    b2.cross(ab, b2);

                    double m0 = b0.lengthSquared();
                    double m1 = b1.lengthSquared();
                    double m2 = b2.lengthSquared();

                    Quat4d tmpQuat = Stack.newQuat();
                    tmp.normalize(ab);
                    QuaternionUtil.setRotation(tmpQuat, tmp, cst2Pi / 3f);

                    Matrix3d r = Stack.newMat();
                    MatrixUtil.setRotation(r, tmpQuat);

                    Vector3d w = Stack.newVec();
                    w.set(m0 > m1 ? m0 > m2 ? b0 : b2 : m1 > m2 ? b1 : b2);

                    tmp.normalize(w);
                    Support(tmp, simplex[4]);
                    r.transform(w);
                    tmp.normalize(w);
                    Support(tmp, simplex[2]);
                    r.transform(w);
                    tmp.normalize(w);
                    Support(tmp, simplex[3]);
                    r.transform(w);
                    order = 4;
                    Stack.subVec(6);
                    Stack.subQuat(1);
                    Stack.subMat(1);
                    return (true);
                }
                // Triangle
                case 2: {
                    Vector3d tmp = Stack.newVec();
                    Vector3d tmp1 = Stack.newVec();
                    Vector3d tmp2 = Stack.newVec();
                    tmp1.sub(simplex[1].w, simplex[0].w);
                    tmp2.sub(simplex[2].w, simplex[0].w);
                    Vector3d n = Stack.newVec();
                    n.cross(tmp1, tmp2);
                    n.normalize();

                    Support(n, simplex[3]);

                    tmp.negate(n);
                    Support(tmp, simplex[4]);
                    order = 4;
                    Stack.subVec(4);
                    return (true);
                }
                // Tetrahedron
                case 3:
                    return (true);
                // Hexahedron
                case 4:
                    return (true);
            }
            return (false);
        }

    }

    ////////////////////////////////////////////////////////////////////////////

    private static int[] mod3 = new int[]{0, 1, 2, 0, 1};

    private static final int[][] tetrahedron_fidx/*[4][3]*/ = new int[][]{{2, 1, 0}, {3, 0, 1}, {3, 1, 2}, {3, 2, 0}};
    private static final int[][] tetrahedron_eidx/*[6][4]*/ = new int[][]{{0, 0, 2, 1}, {0, 1, 1, 1}, {0, 2, 3, 1}, {1, 0, 3, 2}, {2, 0, 1, 2}, {3, 0, 2, 2}};

    private static final int[][] hexahedron_fidx/*[6][3]*/ = new int[][]{{2, 0, 4}, {4, 1, 2}, {1, 4, 0}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    private static final int[][] hexahedron_eidx/*[9][4]*/ = new int[][]{{0, 0, 4, 0}, {0, 1, 2, 1}, {0, 2, 1, 2}, {1, 1, 5, 2}, {1, 0, 2, 0}, {2, 2, 3, 2}, {3, 1, 5, 0}, {3, 0, 4, 2}, {5, 1, 4, 1}};

    public static class Face {
        public final Mkv[] v = new Mkv[3];
        public final Face[] f = new Face[3];
        public final int[] e = new int[3];
        public final Vector3d n = new Vector3d();
        public double d;
        public int mark;
        public Face prev;
        public Face next;
    }

    protected class EPA {
        //protected final BulletStack stack = BulletStack.get();

        public GJK gjk;
        //public btStackAlloc* sa;
        public Face root;
        public int nfaces;
        public int iterations;
        public final Vector3d[][] features = new Vector3d[2][3];
        public final Vector3d[] nearest/*[2]*/ = new Vector3d[]{new Vector3d(), new Vector3d()};
        public final Vector3d normal = new Vector3d();
        public double depth;
        public boolean failed;

        {
            for (int i = 0; i < features.length; i++) {
                for (int j = 0; j < features[i].length; j++) {
                    features[i][j] = new Vector3d();
                }
            }
        }

        public EPA(GJK pgjk) {
            gjk = pgjk;
            //sa = pgjk->sa;
        }

        public Vector3d GetCoordinates(Face face, Vector3d out) {

            Vector3d tmp = Stack.newVec();
            Vector3d tmp1 = Stack.newVec();
            Vector3d tmp2 = Stack.newVec();

            Vector3d o = Stack.newVec();
            o.scale(-face.d, face.n);

            double[] a = doubleArrays.getFixed(3);

            tmp1.sub(face.v[0].w, o);
            tmp2.sub(face.v[1].w, o);
            tmp.cross(tmp1, tmp2);
            a[0] = tmp.length();

            tmp1.sub(face.v[1].w, o);
            tmp2.sub(face.v[2].w, o);
            tmp.cross(tmp1, tmp2);
            a[1] = tmp.length();

            tmp1.sub(face.v[2].w, o);
            tmp2.sub(face.v[0].w, o);
            tmp.cross(tmp1, tmp2);
            a[2] = tmp.length();

            double sm = a[0] + a[1] + a[2];

            out.set(a[1], a[2], a[0]);
            out.scale(1.0 / (sm > 0f ? sm : 1f));

            doubleArrays.release(a);

            Stack.subVec(4);

            return out;
        }

        public Face FindBest() {
            Face bf = null;
            if (root != null) {
                Face cf = root;
                double bd = cstInf;
                do {
                    if (cf.d < bd) {
                        bd = cf.d;
                        bf = cf;
                    }
                }
                while (null != (cf = cf.next));
            }
            return bf;
        }

        public boolean Set(Face f, Mkv a, Mkv b, Mkv c) {

            Vector3d tmp1 = Stack.newVec();
            Vector3d tmp2 = Stack.newVec();
            Vector3d tmp3 = Stack.newVec();

            Vector3d nrm = Stack.newVec();
            tmp1.sub(b.w, a.w);
            tmp2.sub(c.w, a.w);
            nrm.cross(tmp1, tmp2);

            double len = nrm.length();

            tmp1.cross(a.w, b.w);
            tmp2.cross(b.w, c.w);
            tmp3.cross(c.w, a.w);

            boolean valid = (tmp1.dot(nrm) >= -EPA_inface_eps) &&
                    (tmp2.dot(nrm) >= -EPA_inface_eps) &&
                    (tmp3.dot(nrm) >= -EPA_inface_eps);

            f.v[0] = a;
            f.v[1] = b;
            f.v[2] = c;
            f.mark = 0;
            f.n.scale(1.0 / (len > 0f ? len : cstInf), nrm);
            f.d = Math.max(0, -f.n.dot(a.w));

            Stack.subVec(4);

            return valid;

        }

        public Face NewFace(Mkv a, Mkv b, Mkv c) {
            //Face pf = new Face();
            Face pf = stackFace.get();
            if (Set(pf, a, b, c)) {
                if (root != null) {
                    root.prev = pf;
                }
                pf.prev = null;
                pf.next = root;
                root = pf;
                ++nfaces;
            } else {
                pf.prev = pf.next = null;
            }
            return (pf);
        }

        public void Detach(Face face) {
            if (face.prev != null || face.next != null) {
                --nfaces;
                if (face == root) {
                    root = face.next;
                    root.prev = null;
                } else {
                    if (face.next == null) {
                        face.prev.next = null;
                    } else {
                        face.prev.next = face.next;
                        face.next.prev = face.prev;
                    }
                }
                face.prev = face.next = null;
            }
        }

        public void Link(Face f0, int e0, Face f1, int e1) {
            f0.f[e0] = f1;
            f1.e[e1] = e0;
            f1.f[e1] = f0;
            f0.e[e0] = e1;
        }

        public Mkv Support(Vector3d w) {
            //Mkv v = new Mkv();
            Mkv v = stackMkv.get();
            gjk.Support(w, v);
            return v;
        }

        public int BuildHorizon(int markId, Mkv w, Face f, int e, Face[] cf, Face[] ff) {
            int ne = 0;
            if (f.mark != markId) {
                int e1 = mod3[e + 1];
                if ((f.n.dot(w.w) + f.d) > 0) {
                    Face nf = NewFace(f.v[e1], f.v[e], w);
                    Link(nf, 0, f, e);
                    if (cf[0] != null) {
                        Link(cf[0], 1, nf, 2);
                    } else {
                        ff[0] = nf;
                    }
                    cf[0] = nf;
                    ne = 1;
                } else {
                    int e2 = mod3[e + 2];
                    Detach(f);
                    f.mark = markId;
                    ne += BuildHorizon(markId, w, f.f[e1], f.e[e1], cf, ff);
                    ne += BuildHorizon(markId, w, f.f[e2], f.e[e2], cf, ff);
                }
            }
            return (ne);
        }

        public double EvaluatePD() {
            return EvaluatePD(EPA_accuracy);
        }

        public double EvaluatePD(double accuracy) {
            pushStack();
            try {

                //btBlock* sablock = sa->beginBlock();
                Face bestface = null;
                int markid = 1;
                depth = -cstInf;
                normal.set(0.0, 0.0, 0.0);
                root = null;
                nfaces = 0;
                iterations = 0;
                failed = false;
                /* Prepare hull		*/
                if (gjk.EncloseOrigin()) {
                    //const U* pfidx = 0;
                    int[][] pfidx_ptr = null;
                    int pfidx_index = 0;

                    int nfidx = 0;
                    //const U* peidx = 0;
                    int[][] peidx_ptr = null;
                    int peidx_index = 0;

                    int neidx = 0;
                    Mkv[] basemkv = new Mkv[5];
                    Face[] basefaces = new Face[6];
                    switch (gjk.order) {
                        // Tetrahedron
                        case 3: {
                            //pfidx=(const U*)fidx;
                            pfidx_ptr = tetrahedron_fidx;
                            pfidx_index = 0;

                            nfidx = 4;

                            //peidx=(const U*)eidx;
                            peidx_ptr = tetrahedron_eidx;
                            peidx_index = 0;

                            neidx = 6;
                        }
                        break;
                        // Hexahedron
                        case 4: {
                            //pfidx=(const U*)fidx;
                            pfidx_ptr = hexahedron_fidx;
                            pfidx_index = 0;

                            nfidx = 6;

                            //peidx=(const U*)eidx;
                            peidx_ptr = hexahedron_eidx;
                            peidx_index = 0;

                            neidx = 9;
                        }
                        break;
                    }
                    int i;

                    for (i = 0; i <= gjk.order; ++i) {
                        basemkv[i] = new Mkv();
                        basemkv[i].set(gjk.simplex[i]);
                    }
                    for (i = 0; i < nfidx; ++i, pfidx_index++) {
                        basefaces[i] = NewFace(basemkv[pfidx_ptr[pfidx_index][0]], basemkv[pfidx_ptr[pfidx_index][1]], basemkv[pfidx_ptr[pfidx_index][2]]);
                    }
                    for (i = 0; i < neidx; ++i, peidx_index++) {
                        Link(basefaces[peidx_ptr[peidx_index][0]], peidx_ptr[peidx_index][1], basefaces[peidx_ptr[peidx_index][2]], peidx_ptr[peidx_index][3]);
                    }
                }
                if (0 == nfaces) {
                    //sa->endBlock(sablock);
                    return (depth);
                }
                /* Expand hull		*/
                for (; iterations < EPA_maxiterations; ++iterations) {
                    Face bf = FindBest();
                    if (bf != null) {
                        Vector3d tmp = Stack.newVec();
                        tmp.negate(bf.n);
                        Mkv w = Support(tmp);
                        Stack.subVec(1);
                        double d = bf.n.dot(w.w) + bf.d;
                        bestface = bf;
                        if (d < -accuracy) {
                            Face[] cf = new Face[]{null};
                            Face[] ff = new Face[]{null};
                            int nf = 0;
                            Detach(bf);
                            bf.mark = ++markid;
                            for (int i = 0; i < 3; ++i) {
                                nf += BuildHorizon(markid, w, bf.f[i], bf.e[i], cf, ff);
                            }
                            if (nf <= 2) {
                                break;
                            }
                            Link(cf[0], 1, ff[0], 2);
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                /* Extract contact	*/
                if (bestface != null) {
                    Vector3d b = GetCoordinates(bestface, Stack.newVec());
                    normal.set(bestface.n);
                    depth = Math.max(0, bestface.d);

                    Vector3d tmp1 = Stack.newVec();
                    Vector3d tmp2 = Stack.newVec();
                    Vector3d tmp3 = Stack.newVec();

                    for (int i = 0; i < 2; ++i) {
                        double s = i != 0 ? -1f : 1f;
                        for (int j = 0; j < 3; ++j) {
                            tmp1.scale(s, bestface.v[j].r);
                            gjk.LocalSupport(tmp1, i, features[i][j]);
                        }
                    }

                    tmp1.scale(b.x, features[0][0]);
                    tmp2.scale(b.y, features[0][1]);
                    tmp3.scale(b.z, features[0][2]);
                    VectorUtil.add(nearest[0], tmp1, tmp2, tmp3);

                    tmp1.scale(b.x, features[1][0]);
                    tmp2.scale(b.y, features[1][1]);
                    tmp3.scale(b.z, features[1][2]);

                    VectorUtil.add(nearest[1], tmp1, tmp2, tmp3);

                    Stack.subVec(4);

                } else {
                    failed = true;
                }
                //sa->endBlock(sablock);
                return (depth);
            } finally {
                popStack();
            }
        }

    }

    ////////////////////////////////////////////////////////////////////////////

    private GJK gjk = new GJK();

    public boolean collide(ConvexShape shape0, Transform wtrs0,
                           ConvexShape shape1, Transform wtrs1,
                           double radialMargin/*,
			btStackAlloc* stackAlloc*/,
                           Results results) {

        // Initialize
        results.witnesses[0].set(0.0, 0.0, 0.0);
        results.witnesses[1].set(0.0, 0.0, 0.0);
        results.normal.set(0.0, 0.0, 0.0);
        results.depth = 0;
        results.status = ResultsStatus.Separated;
        results.epa_iterations = 0;
        results.gjk_iterations = 0;
        /* Use GJK to locate origin		*/
        gjk.init(/*stackAlloc,*/
                wtrs0.basis, wtrs0.origin, shape0,
                wtrs1.basis, wtrs1.origin, shape1,
                radialMargin + EPA_accuracy);
        try {
            boolean collide = gjk.SearchOrigin();
            results.gjk_iterations = gjk.iterations + 1;
            if (collide) {
                /* Then EPA for penetration depth	*/
                EPA epa = new EPA(gjk);
                double pd = epa.EvaluatePD();
                results.epa_iterations = epa.iterations + 1;
                if (pd > 0) {
                    results.status = ResultsStatus.Penetrating;
                    results.normal.set(epa.normal);
                    results.depth = pd;
                    results.witnesses[0].set(epa.nearest[0]);
                    results.witnesses[1].set(epa.nearest[1]);
                    return (true);
                } else {
                    if (epa.failed) {
                        results.status = ResultsStatus.EPA_Failed;
                    }
                }
            } else {
                if (gjk.failed) {
                    results.status = ResultsStatus.GJK_Failed;
                }
            }
            return (false);
        } finally {
            gjk.destroy();
        }
    }

}
