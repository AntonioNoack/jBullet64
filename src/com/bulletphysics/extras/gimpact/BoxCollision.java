package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * @author jezek2
 */
class BoxCollision {

    public static final double BOX_PLANE_EPSILON = 0.000001;

    public static boolean absGreater(double a, double b) {
        return Math.abs(a) > b;
    }

    public static double max3(double a, double b, double c) {
        return Math.max(a, Math.max(b, c));
    }

    public static double min3(double a, double b, double c) {
        return Math.min(a, Math.min(b, c));
    }

    public static boolean testCrossEdgeBoxMcr(Vector3d edge, Vector3d absolute_edge, Vector3d pointA, Vector3d pointB,
                                              Vector3d extend, int iDir0, int iDir1, int iComp0, int iComp1) {
        double dir0 = -VectorUtil.getCoord(edge, iDir0);
        double dir1 = VectorUtil.getCoord(edge, iDir1);
        double pmin = VectorUtil.getCoord(pointA, iComp0) * dir0 + VectorUtil.getCoord(pointA, iComp1) * dir1;
        double pmax = VectorUtil.getCoord(pointB, iComp0) * dir0 + VectorUtil.getCoord(pointB, iComp1) * dir1;
        if (pmin > pmax) {
            //BT_SWAP_NUMBERS(pmin,pmax);
            pmin = pmin + pmax;
            pmax = pmin - pmax;
            pmin = pmin - pmax;
        }
        double abs_dir0 = VectorUtil.getCoord(absolute_edge, iDir0);
        double abs_dir1 = VectorUtil.getCoord(absolute_edge, iDir1);
        double rad = VectorUtil.getCoord(extend, iComp0) * abs_dir0 + VectorUtil.getCoord(extend, iComp1) * abs_dir1;
        return !(pmin > rad) && !(-rad > pmax);
    }

    public static boolean testCrossEdgeBoxXAxisMcr(Vector3d edge, Vector3d absoluteEdge, Vector3d pointA, Vector3d pointB, Vector3d _extend) {
        return testCrossEdgeBoxMcr(edge, absoluteEdge, pointA, pointB, _extend, 2, 1, 1, 2);
    }

    public static boolean testCrossEdgeBoxYAxisMcr(Vector3d edge, Vector3d absoluteEdge, Vector3d pointA, Vector3d pointB, Vector3d extend) {
        return testCrossEdgeBoxMcr(edge, absoluteEdge, pointA, pointB, extend, 0, 2, 2, 0);
    }

    public static boolean testCrossEdgeBoxZAxisMcr(Vector3d edge, Vector3d absoluteEdge, Vector3d pointA, Vector3d pointB, Vector3d _extend) {
        return testCrossEdgeBoxMcr(edge, absoluteEdge, pointA, pointB, _extend, 1, 0, 0, 1);
    }

    /**
     * Returns the dot product between a vec3 and the col of a matrix.
     */
    public static double max3DotCol(Matrix3d mat, Vector3d vec3, int colIndex) {
        return vec3.x * mat.getElement(0, colIndex) + vec3.y * mat.getElement(1, colIndex) + vec3.z * mat.getElement(2, colIndex);
    }

    /**
     * Compairison of transformation objects.
     */
    public static boolean compareTransformsEqual(Transform t1, Transform t2) {
        return t1.equals(t2);
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class BoxBoxTransformCache {
        public final Vector3d T1to0 = new Vector3d(); // Transforms translation of model1 to model 0
        public final Matrix3d R1to0 = new Matrix3d(); // Transforms Rotation of model1 to model 0, equal  to R0' * R1
        public final Matrix3d AR = new Matrix3d();    // Absolute value of m_R1to0

        public void set(BoxBoxTransformCache cache) {
            throw new UnsupportedOperationException();
        }

        public void calcAbsoluteMatrix() {
            //static const btVector3 vepsi(1e-6f,1e-6f,1e-6f);
            //m_AR[0] = vepsi + m_R1to0[0].absolute();
            //m_AR[1] = vepsi + m_R1to0[1].absolute();
            //m_AR[2] = vepsi + m_R1to0[2].absolute();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    AR.setElement(i, j, 1e-6 + Math.abs(R1to0.getElement(i, j)));
                }
            }
        }

        /**
         * Calc the transformation relative  1 to 0. Inverts matrices by transposing.
         */
        public void calcFromHomogenic(Transform trans0, Transform trans1) {
            Transform temp_trans = Stack.newTrans();
            temp_trans.inverse(trans0);
            temp_trans.mul(trans1);

            T1to0.set(temp_trans.origin);
            R1to0.set(temp_trans.basis);

            calcAbsoluteMatrix();
        }

        /**
         * Calculates the full inversion of the matrices. Useful for scaling matrices.
         */
        public void calcFromFullInvert(Transform trans0, Transform trans1) {
            R1to0.invert(trans0.basis);
            T1to0.negate(trans0.origin);
            R1to0.transform(T1to0);

            Vector3d tmp = Stack.newVec();
            tmp.set(trans1.origin);
            R1to0.transform(tmp);
            T1to0.add(tmp);

            R1to0.mul(trans1.basis);

            calcAbsoluteMatrix();
        }

        public Vector3d transform(Vector3d point, Vector3d out) {
            if (point == out) {
                point = Stack.newVec(point);
            }

            Vector3d tmp = Stack.newVec();
            R1to0.getRow(0, tmp);
            out.x = tmp.dot(point) + T1to0.x;
            R1to0.getRow(1, tmp);
            out.y = tmp.dot(point) + T1to0.y;
            R1to0.getRow(2, tmp);
            out.z = tmp.dot(point) + T1to0.z;
            return out;
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class AABB {
        public final Vector3d min = new Vector3d();
        public final Vector3d max = new Vector3d();

        public AABB() {
        }

        public AABB(Vector3d V1, Vector3d V2, Vector3d V3) {
            calcFromTriangle(V1, V2, V3);
        }

        public AABB(Vector3d V1, Vector3d V2, Vector3d V3, double margin) {
            calcFromTriangleMargin(V1, V2, V3, margin);
        }

        public AABB(AABB other) {
            set(other);
        }

        public AABB(AABB other, double margin) {
            this(other);
            min.x -= margin;
            min.y -= margin;
            min.z -= margin;
            max.x += margin;
            max.y += margin;
            max.z += margin;
        }

        public void init(Vector3d V1, Vector3d V2, Vector3d V3, double margin) {
            calcFromTriangleMargin(V1, V2, V3, margin);
        }

        public void set(AABB other) {
            min.set(other.min);
            max.set(other.max);
        }

        public void invalidate() {
            min.set(BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY);
            max.set(-BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY);
        }

        public void incrementMargin(double margin) {
            min.x -= margin;
            min.y -= margin;
            min.z -= margin;
            max.x += margin;
            max.y += margin;
            max.z += margin;
        }

        public void copyWithMargin(AABB other, double margin) {
            min.x = other.min.x - margin;
            min.y = other.min.y - margin;
            min.z = other.min.z - margin;

            max.x = other.max.x + margin;
            max.y = other.max.y + margin;
            max.z = other.max.z + margin;
        }

        public void calcFromTriangle(Vector3d V1, Vector3d V2, Vector3d V3) {
            min.x = min3(V1.x, V2.x, V3.x);
            min.y = min3(V1.y, V2.y, V3.y);
            min.z = min3(V1.z, V2.z, V3.z);

            max.x = max3(V1.x, V2.x, V3.x);
            max.y = max3(V1.y, V2.y, V3.y);
            max.z = max3(V1.z, V2.z, V3.z);
        }

        public void calcFromTriangleMargin(Vector3d V1, Vector3d V2, Vector3d V3, double margin) {
            calcFromTriangle(V1, V2, V3);
            min.x -= margin;
            min.y -= margin;
            min.z -= margin;
            max.x += margin;
            max.y += margin;
            max.z += margin;
        }

        /**
         * Apply a transform to an AABB.
         */
        public void applyTransform(Transform trans) {
            Vector3d tmp = Stack.newVec();

            Vector3d center = Stack.newVec();
            center.add(max, min);
            center.scale(0.5);

            Vector3d extends_ = Stack.newVec();
            extends_.sub(max, center);

            // Compute new center
            trans.transform(center);

            Vector3d textends = Stack.newVec();

            trans.basis.getRow(0, tmp);
            tmp.absolute();
            textends.x = extends_.dot(tmp);

            trans.basis.getRow(1, tmp);
            tmp.absolute();
            textends.y = extends_.dot(tmp);

            trans.basis.getRow(2, tmp);
            tmp.absolute();
            textends.z = extends_.dot(tmp);

            min.sub(center, textends);
            max.add(center, textends);

            Stack.subVec(4);

        }

        /**
         * Apply a transform to an AABB.
         */
        public void applyTransformTransCache(BoxBoxTransformCache trans) {
            Vector3d tmp = Stack.newVec();

            Vector3d center = Stack.newVec();
            center.add(max, min);
            center.scale(0.5);

            Vector3d extends_ = Stack.newVec();
            extends_.sub(max, center);

            // Compute new center
            trans.transform(center, center);

            Vector3d textends = Stack.newVec();

            trans.R1to0.getRow(0, tmp);
            tmp.absolute();
            textends.x = extends_.dot(tmp);

            trans.R1to0.getRow(1, tmp);
            tmp.absolute();
            textends.y = extends_.dot(tmp);

            trans.R1to0.getRow(2, tmp);
            tmp.absolute();
            textends.z = extends_.dot(tmp);

            min.sub(center, textends);
            max.add(center, textends);

            Stack.subVec(4);
        }

        /**
         * Merges a Box.
         */
        public void merge(AABB box) {
            min.x = Math.min(min.x, box.min.x);
            min.y = Math.min(min.y, box.min.y);
            min.z = Math.min(min.z, box.min.z);

            max.x = Math.max(max.x, box.max.x);
            max.y = Math.max(max.y, box.max.y);
            max.z = Math.max(max.z, box.max.z);
        }

        /**
         * Merges a point.
         */
        public void mergePoint(Vector3d point) {
            min.x = Math.min(min.x, point.x);
            min.y = Math.min(min.y, point.y);
            min.z = Math.min(min.z, point.z);

            max.x = Math.max(max.x, point.x);
            max.y = Math.max(max.y, point.y);
            max.z = Math.max(max.z, point.z);
        }

        /**
         * Gets the extend and center.
         */
        public void getCenterExtend(Vector3d center, Vector3d extend) {
            center.add(max, min);
            center.scale(0.5);

            extend.sub(max, center);
        }

        /**
         * Finds the intersecting box between this box and the other.
         */
        public void findIntersection(AABB other, AABB intersection) {
            intersection.min.x = Math.max(other.min.x, min.x);
            intersection.min.y = Math.max(other.min.y, min.y);
            intersection.min.z = Math.max(other.min.z, min.z);

            intersection.max.x = Math.min(other.max.x, max.x);
            intersection.max.y = Math.min(other.max.y, max.y);
            intersection.max.z = Math.min(other.max.z, max.z);
        }

        public boolean hasCollision(AABB other) {
            return !(min.x > other.max.x || max.x < other.min.x ||
                    min.y > other.max.y || max.y < other.min.y ||
                    min.z > other.max.z || max.z < other.min.z);
        }

        /**
         * Finds the Ray intersection parameter.
         *
         * @param vorigin a vec3f with the origin of the ray
         * @param vdir    a vec3f with the direction of the ray
         */
        public boolean collide_ray(Vector3d vorigin, Vector3d vdir) {
            Vector3d extents = Stack.newVec(), center = Stack.newVec();
            getCenterExtend(center, extents);

            double Dx = vorigin.x - center.x;
            if (absGreater(Dx, extents.x) && Dx * vdir.x >= 0.0f) return false;

            double Dy = vorigin.y - center.y;
            if (absGreater(Dy, extents.y) && Dy * vdir.y >= 0.0f) return false;

            double Dz = vorigin.z - center.z;
            if (absGreater(Dz, extents.z) && Dz * vdir.z >= 0.0f) return false;

            double f = vdir.y * Dz - vdir.z * Dy;
            if (Math.abs(f) > extents.y * Math.abs(vdir.z) + extents.z * Math.abs(vdir.y)) return false;

            f = vdir.z * Dx - vdir.x * Dz;
            if (Math.abs(f) > extents.x * Math.abs(vdir.z) + extents.z * Math.abs(vdir.x)) return false;

            f = vdir.x * Dy - vdir.y * Dx;
            if (Math.abs(f) > extents.x * Math.abs(vdir.y) + extents.y * Math.abs(vdir.x)) return false;

            return true;
        }

        public void projection_interval(Vector3d direction, double[] vmin, double[] vmax) {
            Vector3d tmp = Stack.newVec();

            Vector3d center = Stack.newVec();
            Vector3d extend = Stack.newVec();
            getCenterExtend(center, extend);

            double _fOrigin = direction.dot(center);
            tmp.absolute(direction);
            double _fMaximumExtent = extend.dot(tmp);
            vmin[0] = _fOrigin - _fMaximumExtent;
            vmax[0] = _fOrigin + _fMaximumExtent;

            Stack.subVec(3);

        }

        public PlaneIntersectionType plane_classify(Vector4d plane) {
            Vector3d tmp = Stack.newVec();

            double[] _fmin = new double[1], _fmax = new double[1];
            tmp.set(plane.x, plane.y, plane.z);
            projection_interval(tmp, _fmin, _fmax);
            Stack.subVec(1);

            if (plane.w > _fmax[0] + BOX_PLANE_EPSILON) {
                return PlaneIntersectionType.BACK_PLANE; // 0
            }

            if (plane.w + BOX_PLANE_EPSILON >= _fmin[0]) {
                return PlaneIntersectionType.COLLIDE_PLANE; //1
            }

            return PlaneIntersectionType.FRONT_PLANE; //2
        }

        public boolean overlappingTransConservative(AABB box, Transform trans1_to_0) {
            AABB tBox = new AABB(box);
            tBox.applyTransform(trans1_to_0);
            return hasCollision(tBox);
        }

        public boolean overlappingTransConservative2(AABB box, BoxBoxTransformCache trans1_to_0) {
            AABB tBox = new AABB(box);
            tBox.applyTransformTransCache(trans1_to_0);
            return hasCollision(tBox);
        }

        /**
         * transcache is the transformation cache from box to this AABB.
         */
        public boolean overlappingTransCache(AABB box, BoxBoxTransformCache transcache, boolean fulltest) {
            Vector3d tmp = Stack.newVec();

            // Taken from OPCODE
            Vector3d ea = Stack.newVec(), eb = Stack.newVec(); //extends
            Vector3d ca = Stack.newVec(), cb = Stack.newVec(); //extends
            getCenterExtend(ca, ea);
            box.getCenterExtend(cb, eb);

            Vector3d T = Stack.newVec();
            double t, t2;

            // Class I : A's basis vectors
            for (int i = 0; i < 3; i++) {
                transcache.R1to0.getRow(i, tmp);
                VectorUtil.setCoord(T, i, tmp.dot(cb) + VectorUtil.getCoord(transcache.T1to0, i) - VectorUtil.getCoord(ca, i));

                transcache.AR.getRow(i, tmp);
                t = tmp.dot(eb) + VectorUtil.getCoord(ea, i);
                if (absGreater(VectorUtil.getCoord(T, i), t)) {
                    return false;
                }
            }
            // Class II : B's basis vectors
            for (int i = 0; i < 3; i++) {
                t = max3DotCol(transcache.R1to0, T, i);
                t2 = max3DotCol(transcache.AR, ea, i) + VectorUtil.getCoord(eb, i);
                if (absGreater(t, t2)) {
                    return false;
                }
            }
            // Class III : 9 cross products
            if (fulltest) {
                int m, n, o, p, q, r;
                for (int i = 0; i < 3; i++) {
                    m = (i + 1) % 3;
                    n = (i + 2) % 3;
                    o = (i == 0) ? 1 : 0;
                    p = (i == 2) ? 1 : 2;
                    for (int j = 0; j < 3; j++) {
                        q = j == 2 ? 1 : 2;
                        r = j == 0 ? 1 : 0;
                        t = VectorUtil.getCoord(T, n) * transcache.R1to0.getElement(m, j) - VectorUtil.getCoord(T, m) * transcache.R1to0.getElement(n, j);
                        t2 = VectorUtil.getCoord(ea, o) * transcache.AR.getElement(p, j) + VectorUtil.getCoord(ea, p) * transcache.AR.getElement(o, j) +
                                VectorUtil.getCoord(eb, r) * transcache.AR.getElement(i, q) + VectorUtil.getCoord(eb, q) * transcache.AR.getElement(i, r);
                        if (absGreater(t, t2)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        /**
         * Simple test for planes.
         */
        public boolean collidePlane(Vector4d plane) {
            PlaneIntersectionType classify = plane_classify(plane);
            return (classify == PlaneIntersectionType.COLLIDE_PLANE);
        }

        /**
         * Test for a triangle, with edges.
         */
        public boolean collideTriangleExact(Vector3d p1, Vector3d p2, Vector3d p3, Vector4d triangle_plane) {
            if (!collidePlane(triangle_plane)) {
                return false;
            }
            Vector3d center = Stack.newVec(), extends_ = Stack.newVec();
            getCenterExtend(center, extends_);

            Vector3d v1 = Stack.newVec();
            v1.sub(p1, center);
            Vector3d v2 = Stack.newVec();
            v2.sub(p2, center);
            Vector3d v3 = Stack.newVec();
            v3.sub(p3, center);

            // First axis
            Vector3d diff = Stack.newVec();
            diff.sub(v2, v1);
            Vector3d abs_diff = Stack.newVec();
            abs_diff.absolute(diff);

            // Test With X axis
            testCrossEdgeBoxXAxisMcr(diff, abs_diff, v1, v3, extends_);
            // Test With Y axis
            testCrossEdgeBoxYAxisMcr(diff, abs_diff, v1, v3, extends_);
            // Test With Z axis
            testCrossEdgeBoxZAxisMcr(diff, abs_diff, v1, v3, extends_);

            diff.sub(v3, v2);
            abs_diff.absolute(diff);

            // Test With X axis
            testCrossEdgeBoxXAxisMcr(diff, abs_diff, v2, v1, extends_);
            // Test With Y axis
            testCrossEdgeBoxYAxisMcr(diff, abs_diff, v2, v1, extends_);
            // Test With Z axis
            testCrossEdgeBoxZAxisMcr(diff, abs_diff, v2, v1, extends_);

            diff.sub(v1, v3);
            abs_diff.absolute(diff);

            // Test With X axis
            testCrossEdgeBoxXAxisMcr(diff, abs_diff, v3, v2, extends_);
            // Test With Y axis
            testCrossEdgeBoxYAxisMcr(diff, abs_diff, v3, v2, extends_);
            // Test With Z axis
            testCrossEdgeBoxZAxisMcr(diff, abs_diff, v3, v2, extends_);

            return true;
        }
    }

}
