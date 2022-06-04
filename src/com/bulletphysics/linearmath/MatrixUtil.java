package com.bulletphysics.linearmath;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.util.ArrayPool;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Utility functions for matrices.
 *
 * @author jezek2
 */
public class MatrixUtil {

    public static void scale(Matrix3d dest, Matrix3d mat, Vector3d s) {
        dest.m00 = mat.m00 * s.x;
        dest.m01 = mat.m01 * s.y;
        dest.m02 = mat.m02 * s.z;
        dest.m10 = mat.m10 * s.x;
        dest.m11 = mat.m11 * s.y;
        dest.m12 = mat.m12 * s.z;
        dest.m20 = mat.m20 * s.x;
        dest.m21 = mat.m21 * s.y;
        dest.m22 = mat.m22 * s.z;
    }

    public static void absolute(Matrix3d mat, Matrix3d dst) {
        dst.m00 = Math.abs(mat.m00);
        dst.m01 = Math.abs(mat.m01);
        dst.m02 = Math.abs(mat.m02);
        dst.m10 = Math.abs(mat.m10);
        dst.m11 = Math.abs(mat.m11);
        dst.m12 = Math.abs(mat.m12);
        dst.m20 = Math.abs(mat.m20);
        dst.m21 = Math.abs(mat.m21);
        dst.m22 = Math.abs(mat.m22);
    }

    public static void absolute(Matrix3d mat) {
        mat.m00 = Math.abs(mat.m00);
        mat.m01 = Math.abs(mat.m01);
        mat.m02 = Math.abs(mat.m02);
        mat.m10 = Math.abs(mat.m10);
        mat.m11 = Math.abs(mat.m11);
        mat.m12 = Math.abs(mat.m12);
        mat.m20 = Math.abs(mat.m20);
        mat.m21 = Math.abs(mat.m21);
        mat.m22 = Math.abs(mat.m22);
    }

    public static double dotX(Matrix3d mat, Vector3d vec) {
        return mat.m00 * vec.x + mat.m01 * vec.y + mat.m02 * vec.z;
    }

    public static double dotY(Matrix3d mat, Vector3d vec) {
        return mat.m10 * vec.x + mat.m11 * vec.y + mat.m12 * vec.z;
    }

    public static double dotZ(Matrix3d mat, Vector3d vec) {
        return mat.m20 * vec.x + mat.m21 * vec.y + mat.m22 * vec.z;
    }

    public static double transposedDotX(Matrix3d mat, Vector3d vec) {
        return mat.m00 * vec.x + mat.m10 * vec.y + mat.m20 * vec.z;
    }

    public static double transposedDotY(Matrix3d mat, Vector3d vec) {
        return mat.m01 * vec.x + mat.m11 * vec.y + mat.m21 * vec.z;
    }

    public static double transposedDotZ(Matrix3d mat, Vector3d vec) {
        return mat.m02 * vec.x + mat.m12 * vec.y + mat.m22 * vec.z;
    }

    public static void transposeTransform(Vector3d dest, Vector3d vec, Matrix3d mat) {
        double x = transposedDotX(mat, vec);
        double y = transposedDotY(mat, vec);
        double z = transposedDotZ(mat, vec);
        dest.x = x;
        dest.y = y;
        dest.z = z;
    }

    public static void setRotation(Matrix3d dest, Quat4d q) {
        double d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        assert (d != 0.0);
        double s = 2f / d;
        double xs = q.x * s, ys = q.y * s, zs = q.z * s;
        double wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
        double xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
        double yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
        dest.m00 = 1.0 - (yy + zz);
        dest.m01 = xy - wz;
        dest.m02 = xz + wy;
        dest.m10 = xy + wz;
        dest.m11 = 1.0 - (xx + zz);
        dest.m12 = yz - wx;
        dest.m20 = xz - wy;
        dest.m21 = yz + wx;
        dest.m22 = 1.0 - (xx + yy);
    }

    public static void getRotation(Matrix3d mat, Quat4d dest) {
        ArrayPool<double[]> doubleArrays = ArrayPool.get(double.class);

        double trace = mat.m00 + mat.m11 + mat.m22;
        double[] temp = doubleArrays.getFixed(4);

        if (trace > 0.0) {
            double s = Math.sqrt(trace + 1.0);
            temp[3] = (s * 0.5f);
            s = 0.5f / s;

            temp[0] = ((mat.m21 - mat.m12) * s);
            temp[1] = ((mat.m02 - mat.m20) * s);
            temp[2] = ((mat.m10 - mat.m01) * s);
        } else {
            int i = mat.m00 < mat.m11 ? (mat.m11 < mat.m22 ? 2 : 1) : (mat.m00 < mat.m22 ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;

            double s = Math.sqrt(mat.getElement(i, i) - mat.getElement(j, j) - mat.getElement(k, k) + 1.0);
            temp[i] = s * 0.5f;
            s = 0.5f / s;

            temp[3] = (mat.getElement(k, j) - mat.getElement(j, k)) * s;
            temp[j] = (mat.getElement(j, i) + mat.getElement(i, j)) * s;
            temp[k] = (mat.getElement(k, i) + mat.getElement(i, k)) * s;
        }
        dest.set(temp[0], temp[1], temp[2], temp[3]);

        doubleArrays.release(temp);
    }

    private static double cofac(Matrix3d mat, int r1, int c1, int r2, int c2) {
        return mat.getElement(r1, c1) * mat.getElement(r2, c2) - mat.getElement(r1, c2) * mat.getElement(r2, c1);
    }

    public static void invert(Matrix3d mat) {
        double co_x = cofac(mat, 1, 1, 2, 2);
        double co_y = cofac(mat, 1, 2, 2, 0);
        double co_z = cofac(mat, 1, 0, 2, 1);

        double det = mat.m00 * co_x + mat.m01 * co_y + mat.m02 * co_z;
        assert (det != 0.0);

        double s = 1.0 / det;
        double m00 = co_x * s;
        double m01 = cofac(mat, 0, 2, 2, 1) * s;
        double m02 = cofac(mat, 0, 1, 1, 2) * s;
        double m10 = co_y * s;
        double m11 = cofac(mat, 0, 0, 2, 2) * s;
        double m12 = cofac(mat, 0, 2, 1, 0) * s;
        double m20 = co_z * s;
        double m21 = cofac(mat, 0, 1, 2, 0) * s;
        double m22 = cofac(mat, 0, 0, 1, 1) * s;

        mat.m00 = m00;
        mat.m01 = m01;
        mat.m02 = m02;
        mat.m10 = m10;
        mat.m11 = m11;
        mat.m12 = m12;
        mat.m20 = m20;
        mat.m21 = m21;
        mat.m22 = m22;
    }

    /**
     * Diagonalizes this matrix by the Jacobi method. rot stores the rotation
     * from the coordinate system in which the matrix is diagonal to the original
     * coordinate system, i.e., old_this = rot * new_this * rot^T. The iteration
     * stops when all off-diagonal elements are less than the threshold multiplied
     * by the sum of the absolute values of the diagonal, or when maxSteps have
     * been executed. Note that this matrix is assumed to be symmetric.
     */
    // JAVA NOTE: diagonalize method from 2.71
    public static void diagonalize(Matrix3d mat, Matrix3d rot, double threshold, int maxSteps) {
        Vector3d row = new Vector3d();

        rot.setIdentity();
        for (int step = maxSteps; step > 0; step--) {
            // find off-diagonal element [p][q] with largest magnitude
            int p = 0;
            int q = 1;
            int r = 2;
            double max = Math.abs(mat.m01);
            double v = Math.abs(mat.m02);
            if (v > max) {
                q = 2;
                r = 1;
                max = v;
            }
            v = Math.abs(mat.m12);
            if (v > max) {
                p = 1;
                q = 2;
                r = 0;
                max = v;
            }

            double t = threshold * (Math.abs(mat.m00) + Math.abs(mat.m11) + Math.abs(mat.m22));
            if (max <= t) {
                if (max <= BulletGlobals.SIMD_EPSILON * t) {
                    return;
                }
                step = 1;
            }

            // compute Jacobi rotation J which leads to a zero for element [p][q]
            double mpq = mat.getElement(p, q);
            double theta = (mat.getElement(q, q) - mat.getElement(p, p)) / (2 * mpq);
            double theta2 = theta * theta;
            double cos;
            double sin;
            if ((theta2 * theta2) < (10f / BulletGlobals.SIMD_EPSILON)) {
                t = (theta >= 0.0) ? 1.0 / (theta + Math.sqrt(1f + theta2))
                        : 1.0 / (theta - Math.sqrt(1f + theta2));
                cos = 1.0 / Math.sqrt(1f + t * t);
                sin = cos * t;
            } else {
                // approximation for large theta-value, i.e., a nearly diagonal matrix
                t = 1 / (theta * (2 + 0.5f / theta2));
                cos = 1 - 0.5f * t * t;
                sin = cos * t;
            }

            // apply rotation to matrix (this = J^T * this * J)
            mat.setElement(p, q, 0.0);
            mat.setElement(q, p, 0.0);
            mat.setElement(p, p, mat.getElement(p, p) - t * mpq);
            mat.setElement(q, q, mat.getElement(q, q) + t * mpq);
            double mrp = mat.getElement(r, p);
            double mrq = mat.getElement(r, q);
            mat.setElement(r, p, cos * mrp - sin * mrq);
            mat.setElement(p, r, cos * mrp - sin * mrq);
            mat.setElement(r, q, cos * mrq + sin * mrp);
            mat.setElement(q, r, cos * mrq + sin * mrp);

            // apply rotation to rot (rot = rot * J)
            for (int i = 0; i < 3; i++) {
                rot.getRow(i, row);

                mrp = VectorUtil.getCoord(row, p);
                mrq = VectorUtil.getCoord(row, q);
                VectorUtil.setCoord(row, p, cos * mrp - sin * mrq);
                VectorUtil.setCoord(row, q, cos * mrq + sin * mrp);
                rot.setRow(i, row);
            }
        }
    }

}
