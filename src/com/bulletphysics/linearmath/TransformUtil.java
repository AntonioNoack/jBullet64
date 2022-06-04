package com.bulletphysics.linearmath;

import com.bulletphysics.BulletGlobals;
import cz.advel.stack.Stack;
import cz.advel.stack.StaticAlloc;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Utility functions for transforms.
 *
 * @author jezek2
 */
public class TransformUtil {

    public static final double SIMD_SQRT12 = 0.7071067811865475244008443621048490;
    public static final double ANGULAR_MOTION_THRESHOLD = 0.5 * BulletGlobals.SIMD_HALF_PI;

    public static double reciprocalSqrt(double x) {
        return 1.0 / Math.sqrt(x);
    }

    public static void planeSpace1(Vector3d n, Vector3d p, Vector3d q) {
        if (Math.abs(n.z) > SIMD_SQRT12) {
            // choose p in y-z plane
            double a = n.y * n.y + n.z * n.z;
            double k = reciprocalSqrt(a);
            p.set(0, -n.z * k, n.y * k);
            // set q = n x p
            q.set(a * k, -n.x * p.z, n.x * p.y);
        } else {
            // choose p in x-y plane
            double a = n.x * n.x + n.y * n.y;
            double k = reciprocalSqrt(a);
            p.set(-n.y * k, n.x * k, 0);
            // set q = n x p
            q.set(-n.z * p.y, n.z * p.x, a * k);
        }
    }

    @StaticAlloc
    public static void integrateTransform(Transform curTrans, Vector3d linvel, Vector3d angvel, double timeStep, Transform predictedTransform) {
        predictedTransform.origin.scaleAdd(timeStep, linvel, curTrans.origin);
//	//#define QUATERNION_DERIVATIVE
//	#ifdef QUATERNION_DERIVATIVE
//		btQuaternion predictedOrn = curTrans.getRotation();
//		predictedOrn += (angvel * predictedOrn) * (timeStep * btScalar(0.5));
//		predictedOrn.normalize();
//	#else
        // Exponential map
        // google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

        Vector3d axis = Stack.borrowVec();
        double fAngle = angvel.length();

        // limit the angular motion
        if (fAngle * timeStep > ANGULAR_MOTION_THRESHOLD) {
            fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
        }

        if (fAngle < 0.001) {
            // use Taylor's expansions of sync function
            axis.scale(0.5 * timeStep - (timeStep * timeStep * timeStep) * (0.020833333333) * fAngle * fAngle, angvel);
        } else {
            // sync(fAngle) = sin(c*fAngle)/t
            axis.scale(Math.sin(0.5 * fAngle * timeStep) / fAngle, angvel);
        }
        Quat4d dorn = Stack.newQuat();
        dorn.set(axis.x, axis.y, axis.z, Math.cos(fAngle * timeStep * 0.5));
        Quat4d orn0 = curTrans.getRotation(Stack.newQuat());

        Quat4d predictedOrn = Stack.newQuat();
        predictedOrn.mul(dorn, orn0);
        predictedOrn.normalize();
// #endif
        predictedTransform.setRotation(predictedOrn);

        Stack.subQuat(3);

    }

    public static void calculateVelocity(Transform transform0, Transform transform1, double timeStep, Vector3d linVel, Vector3d angVel) {
        linVel.sub(transform1.origin, transform0.origin);
        linVel.scale(1.0 / timeStep);

        Vector3d axis = Stack.newVec();
        double[] angle = new double[1];
        calculateDiffAxisAngle(transform0, transform1, axis, angle);
        angVel.scale(angle[0] / timeStep, axis);
        Stack.subVec(1);
    }

    public static void calculateDiffAxisAngle(Transform transform0, Transform transform1, Vector3d axis, double[] angle) {

        Matrix3d tmp = Stack.newMat();
        tmp.set(transform0.basis);
        MatrixUtil.invert(tmp);

        Matrix3d dMat = Stack.newMat();
        dMat.mul(transform1.basis, tmp);

        Quat4d dOrn = Stack.newQuat();
        MatrixUtil.getRotation(dMat, dOrn);

        dOrn.normalize();

        angle[0] = QuaternionUtil.getAngle(dOrn);
        axis.set(dOrn.x, dOrn.y, dOrn.z);

        // check for axis length
        double len = axis.lengthSquared();
        if (len < BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON) {
            axis.set(1.0, 0.0, 0.0);
        } else {
            axis.scale(1.0 / Math.sqrt(len));
        }

        Stack.subMat(2);
        Stack.subQuat(1);

    }

}
