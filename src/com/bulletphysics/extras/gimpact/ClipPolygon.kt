package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ArrayPool;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * @author jezek2
 */
class ClipPolygon {

    public static double distancePointPlane(Vector4d plane, Vector3d point) {
        return VectorUtil.dot3(point, plane) - plane.w;
    }

    /**
     * Vector blending. Takes two vectors a, b, blends them together.
     */
    public static void vecBlend(Vector3d vr, Vector3d va, Vector3d vb, double f) {
        vr.scale(1.0 - f, va);
        vr.scaleAdd(f, vb, vr);
    }

    /**
     * Distance from a 3D plane.
     */
    public static void planeClipPolygonCollect(
            Vector3d point0, Vector3d point1, double dist0, double dist1,
            Vector3d[] clipped, int[] clippedCount) {
        boolean prevClassIf = (dist0 > BulletGlobals.SIMD_EPSILON);
        boolean classIf = (dist1 > BulletGlobals.SIMD_EPSILON);
        if (classIf != prevClassIf) {
            double f = -dist0 / (dist1 - dist0);
            vecBlend(clipped[clippedCount[0]], point0, point1, f);
            clippedCount[0]++;
        }
        if (!classIf) {
            clipped[clippedCount[0]].set(point1);
            clippedCount[0]++;
        }
    }

    /**
     * Clips a polygon by a plane.
     *
     * @return The count of the clipped counts
     */
    public static int planeClipPolygon(
            Vector4d plane, Vector3d[] polygonPoints,
            int polygonPointCount, Vector3d[] clipped) {
        ArrayPool<int[]> intArrays = ArrayPool.get(int.class);

        int[] clippedCount = intArrays.getFixed(1);
        clippedCount[0] = 0;

        // clip first point
        double firstDist = distancePointPlane(plane, polygonPoints[0]);
        if (!(firstDist > BulletGlobals.SIMD_EPSILON)) {
            clipped[clippedCount[0]].set(polygonPoints[0]);
            clippedCount[0]++;
        }

        double olddist = firstDist;
        for (int i = 1; i < polygonPointCount; i++) {
            double dist = distancePointPlane(plane, polygonPoints[i]);

            planeClipPolygonCollect(
                    polygonPoints[i - 1], polygonPoints[i],
                    olddist, dist, clipped, clippedCount);


            olddist = dist;
        }

        // RETURN TO FIRST point

        planeClipPolygonCollect(
                polygonPoints[polygonPointCount - 1], polygonPoints[0],
                olddist, firstDist, clipped, clippedCount);

        int ret = clippedCount[0];
        intArrays.release(clippedCount);
        return ret;
    }

    /**
     * Clips a polygon by a plane.
     *
     * @param clipped must be an array of 16 points.
     * @return the count of the clipped counts
     */
    public static int planeClipTriangle(Vector4d plane, Vector3d point0, Vector3d point1, Vector3d point2, Vector3d[] clipped) {
        ArrayPool<int[]> intArrays = ArrayPool.get(int.class);

        int[] clippedCount = intArrays.getFixed(1);
        clippedCount[0] = 0;

        // clip first point0
        double firstdist = distancePointPlane(plane, point0);
        if (!(firstdist > BulletGlobals.SIMD_EPSILON)) {
            clipped[clippedCount[0]].set(point0);
            clippedCount[0]++;
        }

        // point 1
        double olddist = firstdist;
        double dist = distancePointPlane(plane, point1);

        planeClipPolygonCollect(point0, point1, olddist, dist, clipped, clippedCount);

        olddist = dist;


        // point 2
        dist = distancePointPlane(plane, point2);

        planeClipPolygonCollect(point1, point2, olddist, dist, clipped, clippedCount);
        olddist = dist;


        // RETURN TO FIRST point0
        planeClipPolygonCollect(point2, point0, olddist, firstdist, clipped, clippedCount);

        int ret = clippedCount[0];
        intArrays.release(clippedCount);
        return ret;
    }

}
