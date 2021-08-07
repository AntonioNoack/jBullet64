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

import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Utility functions for axis aligned bounding boxes (AABB).
 *
 * @author jezek2
 */
public class AabbUtil2 {

    public static void aabbExpand(Vector3d aabbMin, Vector3d aabbMax, Vector3d expansionMin, Vector3d expansionMax) {
        aabbMin.add(expansionMin);
        aabbMax.add(expansionMax);
    }

    public static int outcode(Vector3d p, Vector3d halfExtent) {
        return (p.x < -halfExtent.x ? 0x01 : 0x0) |
                (p.x > halfExtent.x ? 0x08 : 0x0) |
                (p.y < -halfExtent.y ? 0x02 : 0x0) |
                (p.y > halfExtent.y ? 0x10 : 0x0) |
                (p.z < -halfExtent.z ? 0x4 : 0x0) |
                (p.z > halfExtent.z ? 0x20 : 0x0);
    }

    public static boolean rayAabb(Vector3d rayFrom, Vector3d rayTo, Vector3d aabbMin, Vector3d aabbMax, double[] param, Vector3d normal) {

        int v3 = Stack.getVecPosition();

        Vector3d aabbHalfExtent = Stack.newVec();
        Vector3d aabbCenter = Stack.newVec();
        Vector3d source = Stack.newVec();
        Vector3d target = Stack.newVec();
        Vector3d r = Stack.newVec();
        Vector3d hitNormal = Stack.newVec();

        aabbHalfExtent.sub(aabbMax, aabbMin);
        aabbHalfExtent.scale(0.5);

        aabbCenter.add(aabbMax, aabbMin);
        aabbCenter.scale(0.5);

        source.sub(rayFrom, aabbCenter);
        target.sub(rayTo, aabbCenter);

        int sourceOutcode = outcode(source, aabbHalfExtent);
        int targetOutcode = outcode(target, aabbHalfExtent);
        if ((sourceOutcode & targetOutcode) == 0x0) {

            double lambda_enter = 0.0;
            double lambda_exit = param[0];
            r.sub(target, source);

            double normSign = 1f;
            hitNormal.set(0.0, 0.0, 0.0);
            int bit = 1;

            for (int j = 0; j < 2; j++) {
                for (int i = 0; i != 3; ++i) {
                    if ((sourceOutcode & bit) != 0) {
                        double lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign) / VectorUtil.getCoord(r, i);
                        if (lambda_enter <= lambda) {
                            lambda_enter = lambda;
                            hitNormal.set(0.0, 0.0, 0.0);
                            VectorUtil.setCoord(hitNormal, i, normSign);
                        }
                    } else if ((targetOutcode & bit) != 0) {
                        double lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign) / VectorUtil.getCoord(r, i);
                        //btSetMin(lambda_exit, lambda);
                        lambda_exit = Math.min(lambda_exit, lambda);
                    }
                    bit <<= 1;
                }
                normSign = -1f;
            }
            if (lambda_enter <= lambda_exit) {
                param[0] = lambda_enter;
                normal.set(hitNormal);
                Stack.resetVec(v3);
                return true;
            }
        }
        Stack.resetVec(v3);
        return false;
    }

    /**
     * Conservative test for overlap between two AABBs.
     */
    public static boolean testAabbAgainstAabb2(Vector3d aabbMin1, Vector3d aabbMax1, Vector3d aabbMin2, Vector3d aabbMax2) {
        boolean overlap;
        overlap = !(aabbMin1.x > aabbMax2.x) && !(aabbMax1.x < aabbMin2.x);
        overlap = !(aabbMin1.z > aabbMax2.z) && !(aabbMax1.z < aabbMin2.z) && overlap;
        overlap = !(aabbMin1.y > aabbMax2.y) && !(aabbMax1.y < aabbMin2.y) && overlap;
        return overlap;
    }

    /**
     * Conservative test for overlap between triangle and AABB.
     */
    public static boolean testTriangleAgainstAabb2(Vector3d[] vertices, Vector3d aabbMin, Vector3d aabbMax) {

        Vector3d p1 = vertices[0];
        Vector3d p2 = vertices[1];
        Vector3d p3 = vertices[2];

        if (Math.min(Math.min(p1.x, p2.x), p3.x) > aabbMax.x) return false;
        if (Math.max(Math.max(p1.x, p2.x), p3.x) < aabbMin.x) return false;

        if (Math.min(Math.min(p1.z, p2.z), p3.z) > aabbMax.z) return false;
        if (Math.max(Math.max(p1.z, p2.z), p3.z) < aabbMin.z) return false;

        if (Math.min(Math.min(p1.y, p2.y), p3.y) > aabbMax.y) return false;
        if (Math.max(Math.max(p1.y, p2.y), p3.y) < aabbMin.y) return false;

        return true;
    }

    public static void transformAabb(Vector3d halfExtents, double margin, Transform t, Vector3d aabbMinOut, Vector3d aabbMaxOut) {

        Vector3d halfExtentsWithMargin = Stack.newVec();
        halfExtentsWithMargin.x = halfExtents.x + margin;
        halfExtentsWithMargin.y = halfExtents.y + margin;
        halfExtentsWithMargin.z = halfExtents.z + margin;

        Matrix3d abs_b = Stack.newMat();
        MatrixUtil.absolute(t.basis, abs_b);

        Vector3d tmp = Stack.newVec();

        Vector3d extent = Stack.newVec();
        abs_b.getRow(0, tmp);
        extent.x = tmp.dot(halfExtentsWithMargin);
        abs_b.getRow(1, tmp);
        extent.y = tmp.dot(halfExtentsWithMargin);
        abs_b.getRow(2, tmp);
        extent.z = tmp.dot(halfExtentsWithMargin);

        Vector3d center = t.origin;
        aabbMinOut.sub(center, extent);
        aabbMaxOut.add(center, extent);

        Stack.subVec(3);
        Stack.subMat(1);

    }

    public static void transformAabb(Vector3d localAabbMin, Vector3d localAabbMax, double margin, Transform trans, Vector3d aabbMinOut, Vector3d aabbMaxOut) {

        assert (localAabbMin.x <= localAabbMax.x);
        assert (localAabbMin.y <= localAabbMax.y);
        assert (localAabbMin.z <= localAabbMax.z);

        Vector3d localHalfExtents = Stack.newVec();
        localHalfExtents.sub(localAabbMax, localAabbMin);
        localHalfExtents.scale(0.5);

        localHalfExtents.x += margin;
        localHalfExtents.y += margin;
        localHalfExtents.z += margin;

        Vector3d localCenter = Stack.newVec();
        localCenter.add(localAabbMax, localAabbMin);
        localCenter.scale(0.5);

        Matrix3d abs_b = Stack.newMat(trans.basis);
        MatrixUtil.absolute(abs_b);

        Vector3d center = Stack.newVec(localCenter);
        trans.transform(center);

        Vector3d extent = Stack.newVec();
        Vector3d tmp = Stack.newVec();

        abs_b.getRow(0, tmp);
        extent.x = tmp.dot(localHalfExtents);
        abs_b.getRow(1, tmp);
        extent.y = tmp.dot(localHalfExtents);
        abs_b.getRow(2, tmp);
        extent.z = tmp.dot(localHalfExtents);

        aabbMinOut.sub(center, extent);
        aabbMaxOut.add(center, extent);

        Stack.subVec(5);
        Stack.subMat(1);

    }

}
