package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import java.util.ArrayList;

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
    public static void vecBlend(Vector3d vr, Vector3d va, Vector3d vb, double blendFactor) {
        vr.scale(1.0 - blendFactor, va);
        vr.scaleAdd(blendFactor, vb, vr);
    }

    /**
     * This function calculates the distance from a 3D plane.
     */
    public static int planeClipPolygonCollect(Vector3d point0, Vector3d point1, double dist0, double dist1, ArrayList<Vector3d> clipped, int clippedCount) {
        boolean prevClassIf = (dist0 > BulletGlobals.SIMD_EPSILON);
        boolean classIf = (dist1 > BulletGlobals.SIMD_EPSILON);
        if (classIf != prevClassIf) {
            double blendFactor = -dist0 / (dist1 - dist0);
            vecBlend(clipped.get(clippedCount), point0, point1, blendFactor);
            clippedCount++;
        }
        if (!classIf) {
            clipped.get(clippedCount).set(point1);
            clippedCount++;
        }
        return clippedCount;
    }

    /**
     * Clips a polygon by a plane.
     *
     * @return The count of the clipped counts
     */
    public static int planeClipPolygon(Vector4d plane, ArrayList<Vector3d> polygonPoints, int polygonPointCount, ArrayList<Vector3d> clipped) {

        int clippedCount = 0;

        // clip first point
        double firstDist = distancePointPlane(plane, polygonPoints.get(0));
        if (!(firstDist > BulletGlobals.SIMD_EPSILON)) {
            clipped.get(clippedCount).set(polygonPoints.get(0));
            clippedCount++;
        }

        double oldDist = firstDist;
        for (int i = 1; i < polygonPointCount; i++) {
            double dist = distancePointPlane(plane, polygonPoints.get(i));

            clippedCount = planeClipPolygonCollect(
                    polygonPoints.get(i - 1), polygonPoints.get(i),
                    oldDist, dist,
                    clipped, clippedCount);


            oldDist = dist;
        }

        // RETURN TO FIRST point

        return planeClipPolygonCollect(
                polygonPoints.get(polygonPointCount - 1), polygonPoints.get(0),
                oldDist, firstDist,
                clipped, clippedCount);

    }

    /**
     * Clips a polygon by a plane.
     *
     * @param clipped must be an array of 16 points.
     * @return the count of the clipped counts
     */
    public static int planeClipTriangle(Vector4d plane, Vector3d point0, Vector3d point1, Vector3d point2, ArrayList<Vector3d> clipped) {

        int clippedCount = 0;

        // clip first point0
        double firstDist = distancePointPlane(plane, point0);
        if (!(firstDist > BulletGlobals.SIMD_EPSILON)) {
            clipped.get(clippedCount).set(point0);
            clippedCount++;
        }

        // point 1
        double oldDist = firstDist;
        double dist = distancePointPlane(plane, point1);

        clippedCount = planeClipPolygonCollect(
                point0, point1,
                oldDist,
                dist,
                clipped,
                clippedCount);

        oldDist = dist;


        // point 2
        dist = distancePointPlane(plane, point2);

        clippedCount = planeClipPolygonCollect(
                point1, point2,
                oldDist,
                dist,
                clipped,
                clippedCount);
        oldDist = dist;


        // RETURN TO FIRST point0
        return planeClipPolygonCollect(
                point2, point0,
                oldDist,
                firstDist,
                clipped,
                clippedCount);

    }

}
