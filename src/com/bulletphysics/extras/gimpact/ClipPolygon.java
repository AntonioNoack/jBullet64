package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectArrayList;

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
    public static void vec_blend(Vector3d vr, Vector3d va, Vector3d vb, double blend_factor) {
        vr.scale(1.0 - blend_factor, va);
        vr.scaleAdd(blend_factor, vb, vr);
    }

    /**
     * This function calcs the distance from a 3D plane.
     */
    public static void plane_clip_polygon_collect(Vector3d point0, Vector3d point1, double dist0, double dist1, ObjectArrayList<Vector3d> clipped, int[] clipped_count) {
        boolean _prevclassif = (dist0 > BulletGlobals.SIMD_EPSILON);
        boolean _classif = (dist1 > BulletGlobals.SIMD_EPSILON);
        if (_classif != _prevclassif) {
            double blendfactor = -dist0 / (dist1 - dist0);
            vec_blend(clipped.getQuick(clipped_count[0]), point0, point1, blendfactor);
            clipped_count[0]++;
        }
        if (!_classif) {
            clipped.getQuick(clipped_count[0]).set(point1);
            clipped_count[0]++;
        }
    }

    /**
     * Clips a polygon by a plane.
     *
     * @return The count of the clipped counts
     */
    public static int planeClipPolygon(Vector4d plane, ObjectArrayList<Vector3d> polygon_points, int polygon_point_count, ObjectArrayList<Vector3d> clipped) {
        ArrayPool<int[]> intArrays = ArrayPool.get(int.class);

        int[] clipped_count = intArrays.getFixed(1);
        clipped_count[0] = 0;

        // clip first point
        double firstdist = distancePointPlane(plane, polygon_points.getQuick(0));
        if (!(firstdist > BulletGlobals.SIMD_EPSILON)) {
            clipped.getQuick(clipped_count[0]).set(polygon_points.getQuick(0));
            clipped_count[0]++;
        }

        double olddist = firstdist;
        for (int i = 1; i < polygon_point_count; i++) {
            double dist = distancePointPlane(plane, polygon_points.getQuick(i));

            plane_clip_polygon_collect(
                    polygon_points.getQuick(i - 1), polygon_points.getQuick(i),
                    olddist,
                    dist,
                    clipped,
                    clipped_count);


            olddist = dist;
        }

        // RETURN TO FIRST point

        plane_clip_polygon_collect(
                polygon_points.getQuick(polygon_point_count - 1), polygon_points.getQuick(0),
                olddist,
                firstdist,
                clipped,
                clipped_count);

        int ret = clipped_count[0];
        intArrays.release(clipped_count);
        return ret;
    }

    /**
     * Clips a polygon by a plane.
     *
     * @param clipped must be an array of 16 points.
     * @return the count of the clipped counts
     */
    public static int planeClipTriangle(Vector4d plane, Vector3d point0, Vector3d point1, Vector3d point2, ObjectArrayList<Vector3d> clipped) {
        ArrayPool<int[]> intArrays = ArrayPool.get(int.class);

        int[] clipped_count = intArrays.getFixed(1);
        clipped_count[0] = 0;

        // clip first point0
        double firstdist = distancePointPlane(plane, point0);
        if (!(firstdist > BulletGlobals.SIMD_EPSILON)) {
            clipped.getQuick(clipped_count[0]).set(point0);
            clipped_count[0]++;
        }

        // point 1
        double olddist = firstdist;
        double dist = distancePointPlane(plane, point1);

        plane_clip_polygon_collect(
                point0, point1,
                olddist,
                dist,
                clipped,
                clipped_count);

        olddist = dist;


        // point 2
        dist = distancePointPlane(plane, point2);

        plane_clip_polygon_collect(
                point1, point2,
                olddist,
                dist,
                clipped,
                clipped_count);
        olddist = dist;


        // RETURN TO FIRST point0
        plane_clip_polygon_collect(
                point2, point0,
                olddist,
                firstdist,
                clipped,
                clipped_count);

        int ret = clipped_count[0];
        intArrays.release(clipped_count);
        return ret;
    }

}
