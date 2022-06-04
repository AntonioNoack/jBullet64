package com.bulletphysics;

/**
 * Bullet statistics and profile support.
 *
 * @author jezek2
 */
public class BulletStats {

    public static int gTotalContactPoints;

    // GjkPairDetector
    // temp globals, to improve GJK/EPA/penetration calculations
    public static int gNumDeepPenetrationChecks = 0;
    public static int gNumGjkChecks = 0;
    public static int gNumSplitImpulseRecoveries = 0;

    public static int gOverlappingPairs = 0;
    public static int gRemovePairs = 0;
    public static int gAddedPairs = 0;
    public static int gFindPairs = 0;

    // DiscreteDynamicsWorld:
    public static int gNumClampedCcdMotions = 0;

    // JAVA NOTE: added for statistics in applet demo
    public static long stepSimulationTime;

}
