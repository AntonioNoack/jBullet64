/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

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

    public static final double SIMDSQRT12 = 0.7071067811865475244008443621048490f;
    public static final double ANGULAR_MOTION_THRESHOLD = 0.5f * BulletGlobals.SIMD_HALF_PI;

    public static double recipSqrt(double x) {
        return 1f / Math.sqrt(x);  /* reciprocal square root */
    }

    public static void planeSpace1(Vector3d n, Vector3d p, Vector3d q) {
        if (Math.abs(n.z) > SIMDSQRT12) {
            // choose p in y-z plane
            double a = n.y * n.y + n.z * n.z;
            double k = recipSqrt(a);
            p.set(0, -n.z * k, n.y * k);
            // set q = n x p
            q.set(a * k, -n.x * p.z, n.x * p.y);
        } else {
            // choose p in x-y plane
            double a = n.x * n.x + n.y * n.y;
            double k = recipSqrt(a);
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

        if (fAngle < 0.001f) {
            // use Taylor's expansions of sync function
            axis.scale(0.5f * timeStep - (timeStep * timeStep * timeStep) * (0.020833333333f) * fAngle * fAngle, angvel);
        } else {
            // sync(fAngle) = sin(c*fAngle)/t
            axis.scale(Math.sin(0.5f * fAngle * timeStep) / fAngle, angvel);
        }
        Quat4d dorn = Stack.newQuat();
        dorn.set(axis.x, axis.y, axis.z, Math.cos(fAngle * timeStep * 0.5f));
        Quat4d orn0 = curTrans.getRotation(Stack.newQuat());

        Quat4d predictedOrn = Stack.newQuat();
        predictedOrn.mul(dorn, orn0);
        predictedOrn.normalize();
//  #endif
        predictedTransform.setRotation(predictedOrn);

        Stack.subQuat(3);

    }

    public static void calculateVelocity(Transform transform0, Transform transform1, double timeStep, Vector3d linVel, Vector3d angVel) {
        linVel.sub(transform1.origin, transform0.origin);
        linVel.scale(1f / timeStep);

        Vector3d axis = new Vector3d();
        double[] angle = new double[1];
        calculateDiffAxisAngle(transform0, transform1, axis, angle);
        angVel.scale(angle[0] / timeStep, axis);
    }

    public static void calculateDiffAxisAngle(Transform transform0, Transform transform1, Vector3d axis, double[] angle) {
// #ifdef USE_QUATERNION_DIFF
//		btQuaternion orn0 = transform0.getRotation();
//		btQuaternion orn1a = transform1.getRotation();
//		btQuaternion orn1 = orn0.farthest(orn1a);
//		btQuaternion dorn = orn1 * orn0.inverse();
// #else
        Matrix3d tmp = Stack.newMat();
        tmp.set(transform0.basis);
        MatrixUtil.invert(tmp);

        Matrix3d dmat = Stack.newMat();
        dmat.mul(transform1.basis, tmp);

        Quat4d dorn = Stack.newQuat();
        MatrixUtil.getRotation(dmat, dorn);
// #endif

        // doubleing point inaccuracy can lead to w component > 1..., which breaks

        dorn.normalize();

        angle[0] = QuaternionUtil.getAngle(dorn);
        axis.set(dorn.x, dorn.y, dorn.z);
        // TODO: probably not needed
        //axis[3] = btScalar(0.);

        // check for axis length
        double len = axis.lengthSquared();
        if (len < BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON) {
            axis.set(1f, 0f, 0f);
        } else {
            axis.scale(1f / Math.sqrt(len));
        }
    }

}
