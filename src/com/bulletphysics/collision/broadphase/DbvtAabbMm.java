
// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class DbvtAabbMm {

    private final Vector3d mi = new Vector3d();
    private final Vector3d mx = new Vector3d();

    public DbvtAabbMm() {
    }

    public DbvtAabbMm(DbvtAabbMm o) {
        set(o);
    }

    public void set(DbvtAabbMm o) {
        mi.set(o.mi);
        mx.set(o.mx);
    }

    public static void swap(DbvtAabbMm p1, DbvtAabbMm p2) {
        Vector3d tmp = Stack.borrowVec();

        tmp.set(p1.mi);
        p1.mi.set(p2.mi);
        p2.mi.set(tmp);

        tmp.set(p1.mx);
        p1.mx.set(p2.mx);
        p2.mx.set(tmp);
    }

    public Vector3d Center(Vector3d out) {
        out.add(mi, mx);
        out.scale(0.5);
        return out;
    }

    public Vector3d Lengths(Vector3d out) {
        out.sub(mx, mi);
        return out;
    }

    public Vector3d Extents(Vector3d out) {
        out.sub(mx, mi);
        out.scale(0.5);
        return out;
    }

    public Vector3d Mins() {
        return mi;
    }

    public Vector3d Maxs() {
        return mx;
    }

    public static DbvtAabbMm FromCE(Vector3d c, Vector3d e, DbvtAabbMm out) {
        out.mi.sub(c, e);
        out.mx.add(c, e);
        return out;
    }

    public static DbvtAabbMm FromCR(Vector3d c, double r, DbvtAabbMm out) {
        Vector3d tmp = Stack.borrowVec();
        tmp.set(r, r, r);
        return FromCE(c, tmp, out);
    }

    public static DbvtAabbMm FromMM(Vector3d mi, Vector3d mx, DbvtAabbMm out) {
        out.mi.set(mi);
        out.mx.set(mx);
        return out;
    }

    //public static  DbvtAabbMm	FromPoints( btVector3* pts,int n);
    //public static  DbvtAabbMm	FromPoints( btVector3** ppts,int n);

    public void Expand(Vector3d e) {
        mi.sub(e);
        mx.add(e);
    }

    public void SignedExpand(Vector3d e) {
        if (e.x > 0) {
            mx.x += e.x;
        } else {
            mi.x += e.x;
        }

        if (e.y > 0) {
            mx.y += e.y;
        } else {
            mi.y += e.y;
        }

        if (e.z > 0) {
            mx.z += e.z;
        } else {
            mi.z += e.z;
        }
    }

    public boolean Contain(DbvtAabbMm a) {
        return ((mi.x <= a.mi.x) &&
                (mi.y <= a.mi.y) &&
                (mi.z <= a.mi.z) &&
                (mx.x >= a.mx.x) &&
                (mx.y >= a.mx.y) &&
                (mx.z >= a.mx.z));
    }

    public int Classify(Vector3d n, double o, int s) {
        Vector3d pi = Stack.newVec();
        Vector3d px = Stack.newVec();

        switch (s) {
            case (0):
                px.set(mi.x, mi.y, mi.z);
                pi.set(mx.x, mx.y, mx.z);
                break;
            case (1):
                px.set(mx.x, mi.y, mi.z);
                pi.set(mi.x, mx.y, mx.z);
                break;
            case (2):
                px.set(mi.x, mx.y, mi.z);
                pi.set(mx.x, mi.y, mx.z);
                break;
            case (1 + 2):
                px.set(mx.x, mx.y, mi.z);
                pi.set(mi.x, mi.y, mx.z);
                break;
            case (4):
                px.set(mi.x, mi.y, mx.z);
                pi.set(mx.x, mx.y, mi.z);
                break;
            case (1 + 4):
                px.set(mx.x, mi.y, mx.z);
                pi.set(mi.x, mx.y, mi.z);
                break;
            case (2 + 4):
                px.set(mi.x, mx.y, mx.z);
                pi.set(mx.x, mi.y, mi.z);
                break;
            case (1 + 2 + 4):
                px.set(mx.x, mx.y, mx.z);
                pi.set(mi.x, mi.y, mi.z);
                break;
        }

        int answer;
        if ((n.dot(px) + o) < 0) {
            answer = -1;
        } else if ((n.dot(pi) + o) >= 0) {
            answer = +1;
        } else answer = 0;
        Stack.subVec(2);
        return answer;
    }

    public double ProjectMinimum(Vector3d v, int signs) {
        Vector3d p = Stack.borrowVec();
        p.set(
                ((signs & 1) == 0 ? mx : mi).x,
                ((signs & 2) == 0 ? mx : mi).y,
                ((signs & 4) == 0 ? mx : mi).z
        );
        return p.dot(v);
    }

    public static boolean Intersect(DbvtAabbMm a, DbvtAabbMm b) {
        return ((a.mi.x <= b.mx.x) &&
                (a.mx.x >= b.mi.x) &&
                (a.mi.y <= b.mx.y) &&
                (a.mx.y >= b.mi.y) &&
                (a.mi.z <= b.mx.z) &&
                (a.mx.z >= b.mi.z));
    }

    public static boolean Intersect(DbvtAabbMm a, DbvtAabbMm b, Transform xform) {

        Vector3d d0 = Stack.newVec();
        Vector3d d1 = Stack.newVec();
        Vector3d tmp = Stack.newVec();

        // JAVA NOTE: check
        b.Center(d0);
        xform.transform(d0);
        d0.sub(a.Center(tmp));

        MatrixUtil.transposeTransform(d1, d0, xform.basis);

        double[] s0 = new double[2];
        double[] s1 = new double[2];
        s1[0] = xform.origin.dot(d0);
        s1[1] = s1[0];

        a.AddSpan(d0, s0, s0);
        b.AddSpan(d1, s1, s1);

        Stack.subVec(3);

        if (s0[0] > s1[1]) {
            return false;
        }
        if (s0[1] < s1[0]) {
            return false;
        }

        return true;
    }

    public static boolean Intersect(DbvtAabbMm a, Vector3d b) {
        return ((b.x >= a.mi.x) &&
                (b.y >= a.mi.y) &&
                (b.z >= a.mi.z) &&
                (b.x <= a.mx.x) &&
                (b.y <= a.mx.y) &&
                (b.z <= a.mx.z));
    }

    public static boolean Intersect(DbvtAabbMm a, Vector3d org, Vector3d invdir, int[] signs) {
        Vector3d[] bounds = new Vector3d[]{a.mi, a.mx};
        double txmin = (bounds[signs[0]].x - org.x) * invdir.x;
        double txmax = (bounds[1 - signs[0]].x - org.x) * invdir.x;
        double tymin = (bounds[signs[1]].y - org.y) * invdir.y;
        double tymax = (bounds[1 - signs[1]].y - org.y) * invdir.y;
        if ((txmin > tymax) || (tymin > txmax)) {
            return false;
        }

        if (tymin > txmin) {
            txmin = tymin;
        }
        if (tymax < txmax) {
            txmax = tymax;
        }
        double tzmin = (bounds[signs[2]].z - org.z) * invdir.z;
        double tzmax = (bounds[1 - signs[2]].z - org.z) * invdir.z;
        if ((txmin > tzmax) || (tzmin > txmax)) {
            return false;
        }

        if (tzmin > txmin) {
            txmin = tzmin;
        }
        if (tzmax < txmax) {
            txmax = tzmax;
        }
        return (txmax > 0);
    }

    public static double Proximity(DbvtAabbMm a, DbvtAabbMm b) {

        int v3 = Stack.getVecPosition();

        Vector3d d = Stack.newVec();
        Vector3d tmp = Stack.newVec();

        d.add(a.mi, a.mx);
        tmp.add(b.mi, b.mx);
        d.sub(tmp);

        double answer = Math.abs(d.x) + Math.abs(d.y) + Math.abs(d.z);

        Stack.resetVec(v3);

        return answer;

    }

    public static void Merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm r) {
        r.mi.x = Math.min(a.mi.x, b.mi.x);
        r.mi.y = Math.min(a.mi.y, b.mi.y);
        r.mi.z = Math.min(a.mi.z, b.mi.z);
    }

    public static boolean NotEqual(DbvtAabbMm a, DbvtAabbMm b) {
        return ((a.mi.x != b.mi.x) ||
                (a.mi.y != b.mi.y) ||
                (a.mi.z != b.mi.z) ||
                (a.mx.x != b.mx.x) ||
                (a.mx.y != b.mx.y) ||
                (a.mx.z != b.mx.z));
    }

    private void AddSpan(Vector3d d, double[] smi, int smi_idx, double[] smx, int smx_idx) {
        for (int i = 0; i < 3; i++) {
            if (VectorUtil.getCoord(d, i) < 0) {
                smi[smi_idx] += VectorUtil.getCoord(mx, i) * VectorUtil.getCoord(d, i);
                smx[smx_idx] += VectorUtil.getCoord(mi, i) * VectorUtil.getCoord(d, i);
            } else {
                smi[smi_idx] += VectorUtil.getCoord(mi, i) * VectorUtil.getCoord(d, i);
                smx[smx_idx] += VectorUtil.getCoord(mx, i) * VectorUtil.getCoord(d, i);
            }
        }
    }

    private void AddSpan(Vector3d d, double[] smi, double[] smx) {
        for (int i = 0; i < 3; i++) {
            if (VectorUtil.getCoord(d, i) < 0) {
                smi[0] += VectorUtil.getCoord(mx, i) * VectorUtil.getCoord(d, i);
                smx[1] += VectorUtil.getCoord(mi, i) * VectorUtil.getCoord(d, i);
            } else {
                smi[0] += VectorUtil.getCoord(mi, i) * VectorUtil.getCoord(d, i);
                smx[1] += VectorUtil.getCoord(mx, i) * VectorUtil.getCoord(d, i);
            }
        }
    }

}
