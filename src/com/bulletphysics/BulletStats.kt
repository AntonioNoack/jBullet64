package com.bulletphysics;

import com.bulletphysics.linearmath.CProfileManager;
import com.bulletphysics.linearmath.Clock;

/**
 * Bullet statistics and profile support.
 *
 * @author jezek2
 */
public class BulletStats {

    public static int totalContactPoints;

    // GjkPairDetector
    // temp globals, to improve GJK/EPA/penetration calculations
    public static int numDeepPenetrationChecks = 0;
    public static int numGjkChecks = 0;
    public static int numSplitImpulseRecoveries = 0;

    public static int overlappingPairs = 0;
    public static int removedPairs = 0;
    public static int addedPairs = 0;
    public static int findPairCalls = 0;

    public static final Clock profileClock = new Clock();

    // DiscreteDynamicsWorld:
    public static int numClampedCcdMotions = 0;

    // JAVA NOTE: added for statistics in applet demo
    public static long stepSimulationTime;

    private static boolean enableProfiling = false;

    ////////////////////////////////////////////////////////////////////////////

    public static boolean isProfileEnabled() {
        return enableProfiling;
    }

    public static void setProfileEnabled(boolean b) {
        enableProfiling = b;
    }

    public static long profileGetTicks() {
        return profileClock.getTimeNanos();
    }

    public static double profileGetTickRate() {
        return 1e6;
    }

    /**
     * Pushes profile node. Use try/finally block to call {@link #popProfile} method.
     *
     * @param name must be {@link String#intern interned} String (not needed for String literals)
     */
    public static void pushProfile(String name) {
        if (enableProfiling) {
            CProfileManager.startProfile(name);
        }
    }

    /**
     * Pops profile node.
     */
    public static void popProfile() {
        if (enableProfiling) {
            CProfileManager.stopProfile();
        }
    }

}
