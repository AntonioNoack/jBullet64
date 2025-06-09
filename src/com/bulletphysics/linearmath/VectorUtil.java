package com.bulletphysics.linearmath;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * Utility functions for vectors.
 *
 * @author jezek2
 */
public class VectorUtil {

    public static int maxAxis(Vector3d v) {
        int maxIndex = -1;
        double maxVal = -1e30;
        if (v.x > maxVal) {
            maxIndex = 0;
            maxVal = v.x;
        }
        if (v.y > maxVal) {
            maxIndex = 1;
            maxVal = v.y;
        }
        if (v.z > maxVal) {
            maxIndex = 2;
        }
        return maxIndex;
    }

    public static int closestAxis4(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);

        int maxIndex = -1;
        double maxVal = Double.NEGATIVE_INFINITY;
        if (x > maxVal) {
            maxIndex = 0;
            maxVal = x;
        }
        if (y > maxVal) {
            maxIndex = 1;
            maxVal = y;
        }
        if (z > maxVal) {
            maxIndex = 2;
            maxVal = z;
        }
        if (w > maxVal) {
            maxIndex = 3;
        }
        return maxIndex;
    }

    public static double getCoord(Vector3d vec, int num) {
        switch (num) {
            case 0:
                return vec.x;
            case 1:
                return vec.y;
            default:
                return vec.z;
        }
    }

    public static void setCoord(Vector3d vec, int num, double value) {
        switch (num) {
            case 0:
                vec.x = value;
                break;
            case 1:
                vec.y = value;
                break;
            default:
                vec.z = value;
        }
    }

    public static void mulCoord(Vector3d vec, int num, double value) {
        switch (num) {
            case 0:
                vec.x *= value;
                break;
            case 1:
                vec.y *= value;
                break;
            default:
                vec.z *= value;
        }
    }

    public static void setInterpolate3(Vector3d dest, Vector3d v0, Vector3d v1, double rt) {
        double s = 1.0 - rt;
        dest.x = s * v0.x + rt * v1.x;
        dest.y = s * v0.y + rt * v1.y;
        dest.z = s * v0.z + rt * v1.z;
        // don't do the unused w component
        //		m_co[3] = s * v0[3] + rt * v1[3];
    }

    public static void add(Vector3d dest, Vector3d v1, Vector3d v2) {
        dest.x = v1.x + v2.x;
        dest.y = v1.y + v2.y;
        dest.z = v1.z + v2.z;
    }

    public static void add(Vector3d dest, Vector3d v1, Vector3d v2, Vector3d v3) {
        dest.x = v1.x + v2.x + v3.x;
        dest.y = v1.y + v2.y + v3.y;
        dest.z = v1.z + v2.z + v3.z;
    }

    public static void add(Vector3d dest, Vector3d v1, Vector3d v2, Vector3d v3, Vector3d v4) {
        dest.x = v1.x + v2.x + v3.x + v4.x;
        dest.y = v1.y + v2.y + v3.y + v4.y;
        dest.z = v1.z + v2.z + v3.z + v4.z;
    }

    public static void mul(Vector3d dest, Vector3d v1, Vector3d v2) {
        dest.x = v1.x * v2.x;
        dest.y = v1.y * v2.y;
        dest.z = v1.z * v2.z;
    }

    public static void div(Vector3d dest, Vector3d v1, Vector3d v2) {
        dest.x = v1.x / v2.x;
        dest.y = v1.y / v2.y;
        dest.z = v1.z / v2.z;
    }

    public static void setMin(Vector3d a, Vector3d b) {
        setMin(a, a, b);
    }

    public static void setMax(Vector3d a, Vector3d b) {
        setMax(a, a, b);
    }

    public static void setMin(Vector3d dst, Vector3d a, Vector3d b) {
        dst.x = Math.min(a.x, b.x);
        dst.y = Math.min(a.y, b.y);
        dst.z = Math.min(a.z, b.z);
    }

    public static void setMax(Vector3d dst, Vector3d a, Vector3d b) {
        dst.x = Math.max(a.x, b.x);
        dst.y = Math.max(a.y, b.y);
        dst.z = Math.max(a.z, b.z);
    }

    public static double dot3(Vector4d v0, Vector3d v1) {
        return (v0.x * v1.x + v0.y * v1.y + v0.z * v1.z);
    }

    public static double dot3(Vector4d v0, Vector4d v1) {
        return (v0.x * v1.x + v0.y * v1.y + v0.z * v1.z);
    }

    public static double dot3(Vector3d v0, Vector4d v1) {
        return (v0.x * v1.x + v0.y * v1.y + v0.z * v1.z);
    }

    public static double lengthSquared3(Vector4d v) {
        return (v.x * v.x + v.y * v.y + v.z * v.z);
    }

    public static void normalize3(Vector4d v) {
        double norm = 1.0 / Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        v.x *= norm;
        v.y *= norm;
        v.z *= norm;
    }

    public static void cross3(Vector3d dest, Vector4d v1, Vector4d v2) {
        double x, y;
        x = v1.y * v2.z - v1.z * v2.y;
        y = v2.x * v1.z - v2.z * v1.x;
        dest.z = v1.x * v2.y - v1.y * v2.x;
        dest.x = x;
        dest.y = y;
    }

}
