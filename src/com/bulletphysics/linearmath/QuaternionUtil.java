package com.bulletphysics.linearmath;

import com.bulletphysics.BulletGlobals;
import cz.advel.stack.Stack;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Utility functions for quaternions.
 *
 * @author jezek2
 */
public class QuaternionUtil {

    public static double getAngle(Quat4d q) {
        return 2.0 * Math.acos(q.w);
    }

    public static void setRotation(Quat4d q, Vector3d axis, double angle) {
        double d = axis.length();
        assert (d != 0.0);
        double s = Math.sin(angle * 0.5) / d;
        q.set(axis.x * s, axis.y * s, axis.z * s, Math.cos(angle * 0.5));
    }

    // Game Programming Gems 2.10. make sure v0,v1 are normalized
    public static Quat4d shortestArcQuat(Vector3d v0, Vector3d v1, Quat4d out) {
        Vector3d c = Stack.newVec();
        c.cross(v0, v1);
        double d = v0.dot(v1);

        if (d < -1.0 + BulletGlobals.FLT_EPSILON) {
            // just pick any vector
            out.set(0.0, 1.0, 0.0, 0.0);
            return out;
        }

        double s = Math.sqrt((1.0 + d) * 2.0);
        double rs = 1.0 / s;

        out.set(c.x * rs, c.y * rs, c.z * rs, s * 0.5);
        return out;
    }

    private static void mul(Quat4d q, Vector3d w) {
        double rx = q.w * w.x + q.y * w.z - q.z * w.y;
        double ry = q.w * w.y + q.z * w.x - q.x * w.z;
        double rz = q.w * w.z + q.x * w.y - q.y * w.x;
        double rw = -q.x * w.x - q.y * w.y - q.z * w.z;
        q.set(rx, ry, rz, rw);
    }

    public static Vector3d quatRotate(Quat4d rotation, Vector3d v, Vector3d out) {
        Quat4d q = Stack.newQuat(rotation);
        QuaternionUtil.mul(q, v);

        Quat4d tmp = Stack.newQuat();
        inverse(tmp, rotation);
        q.mul(tmp);

        out.set(q.x, q.y, q.z);
        Stack.subQuat(2);
        return out;
    }

    private static void inverse(Quat4d q, Quat4d src) {
        q.x = -src.x;
        q.y = -src.y;
        q.z = -src.z;
        q.w = src.w;
    }
}
