package com.bulletphysics;

import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

/**
 * Bullet global settings and constants.
 *
 * @author jezek2
 */
@SuppressWarnings("unused")
public class BulletGlobals {

    public static final boolean DEBUG = false;

    public static final double CONVEX_DISTANCE_MARGIN = 0.04;
    // we may have to change that to the correct double value
    public static final double FLT_EPSILON = 1.19209290e-07;
    public static final double SIMD_EPSILON = FLT_EPSILON;

    public static final double SIMD_2_PI = 6.283185307179586232;
    public static final double SIMD_PI = Math.PI;
    public static final double SIMD_HALF_PI = SIMD_2_PI * 0.25;
    public static final double SIMD_RADS_PER_DEG = SIMD_2_PI / 360.0;
    public static final double SIMD_DEGS_PER_RAD = 360.0 / SIMD_2_PI;
    public static final double SIMD_INFINITY = Double.MAX_VALUE;

    ////////////////////////////////////////////////////////////////////////////

    private static final ThreadLocal<BulletGlobals> threadLocal = ThreadLocal.withInitial(BulletGlobals::new);

    private ContactDestroyedCallback gContactDestroyedCallback;
    private ContactAddedCallback gContactAddedCallback;
    private ContactProcessedCallback gContactProcessedCallback;

    private double contactBreakingThreshold = 0.02;
    // RigidBody
    private double deactivationTime = 2;
    private boolean disableDeactivation = false;

    public static ContactAddedCallback getContactAddedCallback() {
        return threadLocal.get().gContactAddedCallback;
    }

    public static void setContactAddedCallback(ContactAddedCallback callback) {
        threadLocal.get().gContactAddedCallback = callback;
    }

    public static ContactDestroyedCallback getContactDestroyedCallback() {
        return threadLocal.get().gContactDestroyedCallback;
    }

    public static void setContactDestroyedCallback(ContactDestroyedCallback callback) {
        threadLocal.get().gContactDestroyedCallback = callback;
    }

    public static ContactProcessedCallback getContactProcessedCallback() {
        return threadLocal.get().gContactProcessedCallback;
    }

    public static void setContactProcessedCallback(ContactProcessedCallback callback) {
        threadLocal.get().gContactProcessedCallback = callback;
    }

    ////////////////////////////////////////////////////////////////////////////

    public static double getContactBreakingThreshold() {
        return threadLocal.get().contactBreakingThreshold;
    }

    public static void setContactBreakingThreshold(double threshold) {
        threadLocal.get().contactBreakingThreshold = threshold;
    }

    public static double getDeactivationTime() {
        return threadLocal.get().deactivationTime;
    }

    public static void setDeactivationTime(double time) {
        threadLocal.get().deactivationTime = time;
    }

    public static boolean isDeactivationDisabled() {
        return threadLocal.get().disableDeactivation;
    }

    public static void setDeactivationDisabled(boolean disable) {
        threadLocal.get().disableDeactivation = disable;
    }

    ////////////////////////////////////////////////////////////////////////////

    /**
     * Cleans all current thread specific settings and caches.
     */
    public static void cleanCurrentThread() {
        threadLocal.remove();
        Stack.libraryCleanCurrentThread();
        ObjectPool.cleanCurrentThread();
        ArrayPool.cleanCurrentThread();
    }

}
