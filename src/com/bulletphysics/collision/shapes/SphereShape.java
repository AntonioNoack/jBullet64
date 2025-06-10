package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * SphereShape implements an implicit sphere, centered around a local origin with radius.
 *
 * @author jezek2
 */
public class SphereShape extends ConvexInternalShape {

    public SphereShape(double radius) {
        setRadius(radius);
    }

    public void setRadius(double radius) {
        implicitShapeDimensions.x = radius;
        collisionMargin = radius;
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d vec, Vector3d out) {
        out.set(0.0, 0.0, 0.0);
        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        for (int i = 0; i < numVectors; i++) {
            supportVerticesOut[i].set(0.0, 0.0, 0.0);
        }
    }

    @Override
    public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        Vector3d center = t.origin;
        Vector3d extent = Stack.borrowVec();
        double margin = getMargin();
        extent.set(margin, margin, margin);
        aabbMin.sub(center, extent);
        aabbMax.add(center, extent);
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.SPHERE_SHAPE_PROXYTYPE;
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        double radius = getMargin();
        double elem = 0.4 * mass * radius * radius;
        inertia.set(elem, elem, elem);
    }

    public double getRadius() {
        return implicitShapeDimensions.x * localScaling.x;
    }

    @Override
    public void setMargin(double margin) {
        super.setMargin(margin);
    }

    @Override
    public double getMargin() {
        // to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
        // this means, non-uniform scaling is not supported anymore
        return getRadius();
    }

}
