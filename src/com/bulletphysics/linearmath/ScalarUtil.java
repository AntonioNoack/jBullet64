package com.bulletphysics.linearmath;

import com.bulletphysics.BulletGlobals;

/**
 * Utility functions for scalars (doubles).
 *
 * @author jezek2
 */
public class ScalarUtil {

    public static double select(double a, double b, double c) {
        return a >= 0 ? b : c;
    }

    public static boolean fuzzyZero(double x) {
        return Math.abs(x) < BulletGlobals.FLT_EPSILON;
    }

    public static double atan2Fast(double y, double x) {
        double coeff1 = BulletGlobals.SIMD_PI / 4.0;
        double coeff2 = 3.0 * coeff1;
        double abs_y = Math.abs(y);
        double angle;
        if (x >= 0.0) {
            double r = (x - abs_y) / (x + abs_y);
            angle = coeff1 - coeff1 * r;
        } else {
            double r = (x + abs_y) / (abs_y - x);
            angle = coeff2 - coeff1 * r;
        }
        return (y < 0.0) ? -angle : angle;
    }

}
