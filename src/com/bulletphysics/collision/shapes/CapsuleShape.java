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

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * CapsuleShape represents a capsule around the Y axis, there is also the
 * {@link CapsuleShapeX} aligned around the X axis and {@link CapsuleShapeZ} around
 * the Z axis.<p>
 * <p>
 * The total height is height+2*radius, so the height is just the height between
 * the center of each "sphere" of the capsule caps.<p>
 * <p>
 * CapsuleShape is a convex hull of two spheres. The {@link MultiSphereShape} is
 * a more general collision shape that takes the convex hull of multiple sphere,
 * so it can also represent a capsule when just using two spheres.
 *
 * @author jezek2
 */
public class CapsuleShape extends ConvexInternalShape {

    public int upAxis;

    // only used for CapsuleShapeZ and CapsuleShapeX subclasses.
    CapsuleShape() {
    }

    public CapsuleShape(double radius, double height) {
        upAxis = 1;
        implicitShapeDimensions.set(radius, 0.5f * height, radius);
    }

    public CapsuleShape(double radius, double height, int upAxis) {
        this.upAxis = upAxis;
        switch (upAxis) {
            case 0:
                implicitShapeDimensions.set(0.5f * height, radius, radius);
                break;
            case 1:
                implicitShapeDimensions.set(radius, 0.5f * height, radius);
                break;
            case 2:
                implicitShapeDimensions.set(radius, radius, 0.5f * height);
                break;
            default:
                throw new IllegalArgumentException("Axis must be 0-2");
        }
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d vec0, Vector3d supVec) {
        supVec.set(0, 0, 0);

        double maxDot = -1e300;

        int v3 = Stack.getVecPosition();

        Vector3d vec = Stack.newVec(vec0);
        double lenSqr = vec.lengthSquared();
        if (lenSqr < 0.0001) {
            vec.set(1, 0, 0);
        } else {
            vec.scale(1.0 / Math.sqrt(lenSqr));
        }

        Vector3d vtx = Stack.newVec();
        double newDot;

        double radius = getRadius();

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();
        Vector3d pos = Stack.newVec();

        {
            pos.set(0.0, 0.0, 0.0);
            VectorUtil.setCoord(pos, getUpAxis(), getHalfHeight());

            VectorUtil.mul(tmp1, vec, localScaling);
            tmp1.scale(radius);
            tmp2.scale(getMargin(), vec);
            vtx.add(pos, tmp1);
            vtx.sub(tmp2);
            newDot = vec.dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                supVec.set(vtx);
            }
        }
        {
            pos.set(0, 0, 0);
            VectorUtil.setCoord(pos, getUpAxis(), -getHalfHeight());

            VectorUtil.mul(tmp1, vec, localScaling);
            tmp1.scale(radius);
            tmp2.scale(getMargin(), vec);
            vtx.add(pos, tmp1);
            vtx.sub(tmp2);
            newDot = vec.dot(vtx);
            if (newDot > maxDot) {
                // maxDot = newDot;
                supVec.set(vtx);
            }
        }

        Stack.resetVec(v3);

        return supVec;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        // TODO: implement
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        // as an approximation, take the inertia of the box that bounds the spheres

        Transform ident = Stack.newTrans();
        ident.setIdentity();

        double radius = getRadius();

        Vector3d halfExtents = Stack.newVec();
        halfExtents.set(radius, radius, radius);
        VectorUtil.setCoord(halfExtents, getUpAxis(), radius + getHalfHeight());

        double margin = BulletGlobals.CONVEX_DISTANCE_MARGIN;

        double lx = 2.0 * (halfExtents.x + margin);
        double ly = 2.0 * (halfExtents.y + margin);
        double lz = 2.0 * (halfExtents.z + margin);
        double x2 = lx * lx;
        double y2 = ly * ly;
        double z2 = lz * lz;
        double scaledMass = mass * 0.0833333333333333333;

        inertia.x = scaledMass * (y2 + z2);
        inertia.y = scaledMass * (x2 + z2);
        inertia.z = scaledMass * (x2 + y2);
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.CAPSULE_SHAPE_PROXYTYPE;
    }

    @Override
    public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        Vector3d tmp = Stack.newVec();

        Vector3d halfExtents = Stack.newVec();
        halfExtents.set(getRadius(), getRadius(), getRadius());
        VectorUtil.setCoord(halfExtents, upAxis, getRadius() + getHalfHeight());

        halfExtents.x += getMargin();
        halfExtents.y += getMargin();
        halfExtents.z += getMargin();

        Matrix3d abs_b = Stack.newMat();
        abs_b.set(t.basis);
        MatrixUtil.absolute(abs_b);

        Vector3d center = t.origin;
        Vector3d extent = Stack.newVec();

        abs_b.getRow(0, tmp);
        extent.x = tmp.dot(halfExtents);
        abs_b.getRow(1, tmp);
        extent.y = tmp.dot(halfExtents);
        abs_b.getRow(2, tmp);
        extent.z = tmp.dot(halfExtents);

        aabbMin.sub(center, extent);
        aabbMax.add(center, extent);
    }

    @Override
    public String getName() {
        return "CapsuleShape";
    }

    public int getUpAxis() {
        return upAxis;
    }

    public double getRadius() {
        int radiusAxis = (upAxis + 2) % 3;
        return VectorUtil.getCoord(implicitShapeDimensions, radiusAxis);
    }

    public double getHalfHeight() {
        return VectorUtil.getCoord(implicitShapeDimensions, upAxis);
    }

}
