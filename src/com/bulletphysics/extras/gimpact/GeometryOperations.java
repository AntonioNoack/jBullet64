package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * @author jezek2
 */
class GeometryOperations {

    public static final double PLANE_DIR_EPSILON = 0.0000001f;

    public static double CLAMP(double number, double minval, double maxval) {
        return (number < minval ? minval : (number > maxval ? maxval : number));
    }

    /**
     * Calc a plane from a triangle edge an a normal.
     */
    public static void edgePlane(Vector3d e1, Vector3d e2, Vector3d normal, Vector4d plane) {
        Vector3d planeNormal = Stack.newVec();
        planeNormal.sub(e2, e1);
        planeNormal.cross(planeNormal, normal);
        planeNormal.normalize();

        plane.set(planeNormal);
        plane.w = e2.dot(planeNormal);
    }

    /**
     * Finds the closest point(cp) to (v) on a segment (e1,e2).
     */
    public static void closestPointOnSegment(Vector3d cp, Vector3d v, Vector3d e1, Vector3d e2) {
        Vector3d n = Stack.borrowVec();
        n.sub(e2, e1);
        cp.sub(v, e1);
        double t = cp.dot(n) / n.dot(n);
        if (t < 0.0) {
            cp.set(e1);
        } else if (t > 1.0) {
            cp.set(e2);
        } else {
            cp.scaleAdd(t, n, e1);
        }
    }

    /**
     * Line plane collision.
     *
     * @return -0 if the ray never intersects, -1 if the ray collides in front, -2 if the ray collides in back
     */
    public static int linePlaneCollision(Vector4d plane, Vector3d vDir, Vector3d vPoint, Vector3d pout, double[] tparam, double tmin, double tmax) {
        double dotDir = VectorUtil.dot3(vDir, plane);

        if (Math.abs(dotDir) < PLANE_DIR_EPSILON) {
            tparam[0] = tmax;
            return 0;
        }

        double dis = ClipPolygon.distancePointPlane(plane, vPoint);
        int returnvalue = dis < 0.0 ? 2 : 1;
        tparam[0] = -dis / dotDir;

        if (tparam[0] < tmin) {
            returnvalue = 0;
            tparam[0] = tmin;
        } else if (tparam[0] > tmax) {
            returnvalue = 0;
            tparam[0] = tmax;
        }
        pout.scaleAdd(tparam[0], vDir, vPoint);
        return returnvalue;
    }

    /**
     * Find closest points on segments.
     */
    public static void segmentCollision(Vector3d vA1, Vector3d vA2, Vector3d vB1, Vector3d vB2, Vector3d vPointA, Vector3d vPointB) {
        Vector3d AD = Stack.newVec();
        AD.sub(vA2, vA1);

        Vector3d BD = Stack.newVec();
        BD.sub(vB2, vB1);

        Vector3d N = Stack.newVec();
        N.cross(AD, BD);
        double[] tp = new double[]{N.lengthSquared()};

        Vector4d _M = new Vector4d();//plane

        if (tp[0] < BulletGlobals.SIMD_EPSILON) {// are parallel

            // project B over A
            boolean invertBOrder = false;
            _M.x = vB1.dot(AD);
            _M.y = vB2.dot(AD);

            if (_M.x > _M.y) {
                invertBOrder = true;
                //BT_SWAP_NUMBERS(_M[0],_M[1]);
                _M.x = _M.x + _M.y;
                _M.y = _M.x - _M.y;
                _M.x = _M.x - _M.y;
            }
            _M.z = vA1.dot(AD);
            _M.w = vA2.dot(AD);
            // mid points
            N.x = (_M.x + _M.y) * 0.5;
            N.y = (_M.z + _M.w) * 0.5;

            if (N.x < N.y) {
                if (_M.y < _M.z) {
                    vPointB = invertBOrder ? vB1 : vB2;
                    vPointA = vA1;
                } else if (_M.y < _M.w) {
                    vPointB = invertBOrder ? vB1 : vB2;
                    closestPointOnSegment(vPointA, vPointB, vA1, vA2);
                } else {
                    vPointA = vA2;
                    closestPointOnSegment(vPointB, vPointA, vB1, vB2);
                }
            } else {
                if (_M.w < _M.x) {
                    vPointB = invertBOrder ? vB2 : vB1;
                    vPointA = vA2;
                } else if (_M.w < _M.y) {
                    vPointA = vA2;
                    closestPointOnSegment(vPointB, vPointA, vB1, vB2);
                } else {
                    vPointB = invertBOrder ? vB1 : vB2;
                    closestPointOnSegment(vPointA, vPointB, vA1, vA2);
                }
            }
            return;
        }

        N.cross(N, BD);
        _M.set(N.x, N.y, N.z, vB1.dot(N));

        // get point A as the plane collision point
        linePlaneCollision(_M, AD, vA1, vPointA, tp, 0.0, 1.0);

        /*Closest point on segment*/
        vPointB.sub(vPointA, vB1);
        tp[0] = vPointB.dot(BD);
        tp[0] /= BD.dot(BD);
        tp[0] = CLAMP(tp[0], 0.0, 1.0);

        vPointB.scaleAdd(tp[0], BD, vB1);
    }

}
