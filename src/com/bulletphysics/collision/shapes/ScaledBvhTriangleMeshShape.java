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

package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

// JAVA NOTE: ScaledBvhTriangleMeshShape from 2.73 SP1

/**
 * The ScaledBvhTriangleMeshShape allows to instance a scaled version of an existing
 * {@link BvhTriangleMeshShape}. Note that each {@link BvhTriangleMeshShape} still can
 * have its own local scaling, independent from this ScaledBvhTriangleMeshShape 'localScaling'.
 *
 * @author jezek2
 */
public class ScaledBvhTriangleMeshShape extends ConcaveShape {

    protected final Vector3d localScaling = new Vector3d();
    protected BvhTriangleMeshShape bvhTriMeshShape;

    public ScaledBvhTriangleMeshShape(BvhTriangleMeshShape childShape, Vector3d localScaling) {
        this.localScaling.set(localScaling);
        this.bvhTriMeshShape = childShape;
    }

    public BvhTriangleMeshShape getChildShape() {
        return bvhTriMeshShape;
    }

    @Override
    public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
        ScaledTriangleCallback scaledCallback = new ScaledTriangleCallback(callback, localScaling);

        Vector3d invLocalScaling = Stack.newVec();
        invLocalScaling.set(1.f / localScaling.x, 1.f / localScaling.y, 1.f / localScaling.z);

        Vector3d scaledAabbMin = Stack.newVec();
        Vector3d scaledAabbMax = Stack.newVec();

        // support negative scaling
        scaledAabbMin.x = localScaling.x >= 0.0 ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
        scaledAabbMin.y = localScaling.y >= 0.0 ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
        scaledAabbMin.z = localScaling.z >= 0.0 ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

        scaledAabbMax.x = localScaling.x <= 0.0 ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
        scaledAabbMax.y = localScaling.y <= 0.0 ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
        scaledAabbMax.z = localScaling.z <= 0.0 ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

        bvhTriMeshShape.processAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax);
    }

    @Override
    public void getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
        Vector3d localAabbMin = bvhTriMeshShape.getLocalAabbMin(Stack.newVec());
        Vector3d localAabbMax = bvhTriMeshShape.getLocalAabbMax(Stack.newVec());

        Vector3d tmpLocalAabbMin = Stack.newVec();
        Vector3d tmpLocalAabbMax = Stack.newVec();
        VectorUtil.mul(tmpLocalAabbMin, localAabbMin, localScaling);
        VectorUtil.mul(tmpLocalAabbMax, localAabbMax, localScaling);

        localAabbMin.x = (localScaling.x >= 0.0) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
        localAabbMin.y = (localScaling.y >= 0.0) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
        localAabbMin.z = (localScaling.z >= 0.0) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;
        localAabbMax.x = (localScaling.x <= 0.0) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
        localAabbMax.y = (localScaling.y <= 0.0) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
        localAabbMax.z = (localScaling.z <= 0.0) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;

        Vector3d localHalfExtents = Stack.newVec();
        localHalfExtents.sub(localAabbMax, localAabbMin);
        localHalfExtents.scale(0.5);

        double margin = bvhTriMeshShape.getMargin();
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

        aabbMin.sub(center, extent);
        aabbMax.add(center, extent);
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        localScaling.set(scaling);
    }

    @Override
    public Vector3d getLocalScaling(Vector3d out) {
        out.set(localScaling);
        return out;
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
    }

    @Override
    public String getName() {
        return "SCALEDBVHTRIANGLEMESH";
    }

    ////////////////////////////////////////////////////////////////////////////

    private static class ScaledTriangleCallback extends TriangleCallback {
        private final TriangleCallback originalCallback;
        private final Vector3d localScaling = new Vector3d();
        private final Vector3d[] newTriangle = new Vector3d[3];

        public ScaledTriangleCallback(TriangleCallback originalCallback, Vector3d localScaling) {
            this.originalCallback = originalCallback;
            this.localScaling.set(localScaling);

            for (int i = 0; i < newTriangle.length; i++) {
                newTriangle[i] = new Vector3d();
            }
        }

        public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
            VectorUtil.mul(newTriangle[0], triangle[0], localScaling);
            VectorUtil.mul(newTriangle[1], triangle[1], localScaling);
            VectorUtil.mul(newTriangle[2], triangle[2], localScaling);
            originalCallback.processTriangle(newTriangle, partId, triangleIndex);
        }
    }

}
