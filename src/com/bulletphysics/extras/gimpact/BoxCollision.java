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

    public static boolean absGreater(double i, double j) {
        return Math.abs(i) > j;
    }

    public static double max(double a, double b, double c) {
        return Math.max(a, Math.max(b, c));
    }

    public static double min(double a, double b, double c) {
        return Math.min(a, Math.min(b, c));
    }

    /**
     * Returns the dot product between a vec3 and the col of a matrix.
     */
    public static double matXVec(Matrix3d mat, Vector3d vec3, int column) {
        return vec3.x * mat.getElement(0, column) + vec3.y * mat.getElement(1, column) + vec3.z * mat.getElement(2, column);
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class BoxBoxTransformCache {
        public final Vector3d T1to0 = new Vector3d(); // Transforms translation of model1 to model 0
        public final Matrix3d R1to0 = new Matrix3d(); // Transforms Rotation of model1 to model 0, equal  to R0' * R1
        public final Matrix3d AR = new Matrix3d();    // Absolute value of m_R1to0

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
         * Calcs the full invertion of the matrices. Useful for scaling matrices.
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

        public void calcFromTriangle(Vector3d V1, Vector3d V2, Vector3d V3) {
            min.x = min(V1.x, V2.x, V3.x);
            min.y = min(V1.y, V2.y, V3.y);
            min.z = min(V1.z, V2.z, V3.z);

            max.x = max(V1.x, V2.x, V3.x);
            max.y = max(V1.y, V2.y, V3.y);
            max.z = max(V1.z, V2.z, V3.z);
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
         * Gets the extend and center.
         */
        public void getCenterExtend(Vector3d center, Vector3d extend) {
            center.add(max, min);
            center.scale(0.5);
            extend.sub(max, center);
        }

        public boolean hasCollision(AABB other) {
            return !(min.x > other.max.x) &&
                    !(max.x < other.min.x) &&
                    !(min.y > other.max.y) &&
                    !(max.y < other.min.y) &&
                    !(min.z > other.max.z) &&
                    !(max.z < other.min.z);
        }

        /**
         * Finds the Ray intersection parameter.
         *
         * @param origin a vec3 with the origin of the ray
         * @param dir    a vec3 with the direction of the ray
         */
        public boolean collideRay(Vector3d origin, Vector3d dir) {
            Vector3d extents = Stack.newVec(), center = Stack.newVec();
            getCenterExtend(center, extents);

            double Dx = origin.x - center.x;
            if (absGreater(Dx, extents.x) && Dx * dir.x >= 0.0) return false;

            double Dy = origin.y - center.y;
            if (absGreater(Dy, extents.y) && Dy * dir.y >= 0.0) return false;

            double Dz = origin.z - center.z;
            if (absGreater(Dz, extents.z) && Dz * dir.z >= 0.0) return false;

            double f = dir.y * Dz - dir.z * Dy;
            if (Math.abs(f) > extents.y * Math.abs(dir.z) + extents.z * Math.abs(dir.y)) return false;

            f = dir.z * Dx - dir.x * Dz;
            if (Math.abs(f) > extents.x * Math.abs(dir.z) + extents.z * Math.abs(dir.x)) return false;

            f = dir.x * Dy - dir.y * Dx;
            if (Math.abs(f) > extents.x * Math.abs(dir.y) + extents.y * Math.abs(dir.x)) return false;

            return true;
        }

        public void projectionInterval(Vector3d direction, double[] vmin, double[] vmax) {
            Vector3d tmp = Stack.newVec();

            Vector3d center = Stack.newVec();
            Vector3d extend = Stack.newVec();
            getCenterExtend(center, extend);

            double _fOrigin = direction.dot(center);
            tmp.absolute(direction);
            double _fMaximumExtent = extend.dot(tmp);
            vmin[0] = _fOrigin - _fMaximumExtent;
            vmax[0] = _fOrigin + _fMaximumExtent;
        }

        public PlaneIntersectionType planeClassify(Vector4d plane) {
            Vector3d tmp = Stack.newVec();

            double[] _fmin = new double[1], _fmax = new double[1];
            tmp.set(plane.x, plane.y, plane.z);
            projectionInterval(tmp, _fmin, _fmax);

            if (plane.w > _fmax[0] + BOX_PLANE_EPSILON) {
                return PlaneIntersectionType.BACK_PLANE; // 0
            }

            if (plane.w + BOX_PLANE_EPSILON >= _fmin[0]) {
                return PlaneIntersectionType.COLLIDE_PLANE; //1
            }

            return PlaneIntersectionType.FRONT_PLANE; //2
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
                t = matXVec(transcache.R1to0, T, i);
                t2 = matXVec(transcache.AR, ea, i) + VectorUtil.getCoord(eb, i);
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
    }
}
