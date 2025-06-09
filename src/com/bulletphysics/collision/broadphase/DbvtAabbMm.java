// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

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

    @SuppressWarnings("UnusedReturnValue")
    public static DbvtAabbMm FromCR(Vector3d c, double r, DbvtAabbMm out) {
        Vector3d tmp = Stack.newVec();
        tmp.set(r, r, r);
        return FromCE(c, tmp, out);
    }

    public static DbvtAabbMm FromMM(Vector3d mi, Vector3d mx, DbvtAabbMm out) {
        out.mi.set(mi);
        out.mx.set(mx);
        return out;
    }

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

    public static boolean Intersect(DbvtAabbMm a, DbvtAabbMm b) {
        return ((a.mi.x <= b.mx.x) &&
                (a.mx.x >= b.mi.x) &&
                (a.mi.y <= b.mx.y) &&
                (a.mx.y >= b.mi.y) &&
                (a.mi.z <= b.mx.z) &&
                (a.mx.z >= b.mi.z));
    }

    public static double Proximity(DbvtAabbMm a, DbvtAabbMm b) {
        Vector3d ai = a.mi, ax = a.mx;
        Vector3d bi = b.mi, bx = b.mx;
        return Math.abs((ai.x + ax.x) - (bi.x + bx.x)) +
                Math.abs((ai.y + ax.y) - (bi.y + bx.y)) +
                Math.abs((ai.z + ax.z) - (bi.z + bx.z));
    }

    public static void Merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm r) {
        VectorUtil.setMin(r.mi, a.mi, b.mi);
        VectorUtil.setMax(r.mx, a.mx, b.mx);
    }

    public static boolean NotEqual(DbvtAabbMm a, DbvtAabbMm b) {
        return ((a.mi.x != b.mi.x) ||
                (a.mi.y != b.mi.y) ||
                (a.mi.z != b.mi.z) ||
                (a.mx.x != b.mx.x) ||
                (a.mx.y != b.mx.y) ||
                (a.mx.z != b.mx.z));
    }

}
