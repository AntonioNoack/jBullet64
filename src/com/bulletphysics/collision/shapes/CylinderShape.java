package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * CylinderShape class implements a cylinder shape primitive, centered around
 * the origin. Its central axis aligned with the Y axis. {@link CylinderShapeX}
 * is aligned with the X axis and {@link CylinderShapeZ} around the Z axis.
 *
 * @author jezek2
 */
public class CylinderShape extends BoxShape {

    protected int upAxis;

    @SuppressWarnings("unused")
    public CylinderShape(Vector3d halfExtents) {
        super(halfExtents);
        upAxis = 1;
        recalculateLocalAabb();
    }

    @Override
    public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        _PolyhedralConvexShape_getAabb(t, aabbMin, aabbMax);
    }

    protected Vector3d cylinderLocalSupportX(Vector3d halfExtents, Vector3d v, Vector3d out) {
        return cylinderLocalSupport(halfExtents, v, 0, 1, 0, 2, out);
    }

    protected Vector3d cylinderLocalSupportY(Vector3d halfExtents, Vector3d v, Vector3d out) {
        return cylinderLocalSupport(halfExtents, v, 1, 0, 1, 2, out);
    }

    protected Vector3d cylinderLocalSupportZ(Vector3d halfExtents, Vector3d v, Vector3d out) {
        return cylinderLocalSupport(halfExtents, v, 2, 0, 2, 1, out);
    }

    private Vector3d cylinderLocalSupport(Vector3d halfExtents, Vector3d v, int cylinderUpAxis, int XX, int YY, int ZZ, Vector3d out) {
        //mapping depends on how cylinder local orientation is
        // extents of the cylinder is: X,Y is for radius, and Z for height

        double radius = VectorUtil.getCoord(halfExtents, XX);
        double halfHeight = VectorUtil.getCoord(halfExtents, cylinderUpAxis);

        double s = Math.sqrt(VectorUtil.getCoord(v, XX) * VectorUtil.getCoord(v, XX) + VectorUtil.getCoord(v, ZZ) * VectorUtil.getCoord(v, ZZ));
        if (s != 0.0) {
            double d = radius / s;
            VectorUtil.setCoord(out, XX, VectorUtil.getCoord(v, XX) * d);
            VectorUtil.setCoord(out, YY, VectorUtil.getCoord(v, YY) < 0.0 ? -halfHeight : halfHeight);
            VectorUtil.setCoord(out, ZZ, VectorUtil.getCoord(v, ZZ) * d);
        } else {
            VectorUtil.setCoord(out, XX, radius);
            VectorUtil.setCoord(out, YY, VectorUtil.getCoord(v, YY) < 0.0 ? -halfHeight : halfHeight);
            VectorUtil.setCoord(out, ZZ, 0.0);
        }
        return out;
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        Vector3d halfExtends = getHalfExtentsWithoutMargin(Stack.newVec());
        Vector3d result = cylinderLocalSupportY(halfExtends, dir, out);
        Stack.subVec(1);
        return result;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        Vector3d halfExtends = getHalfExtentsWithoutMargin(Stack.newVec());
        for (int i = 0; i < numVectors; i++) {
            cylinderLocalSupportY(halfExtends, vectors[i], supportVerticesOut[i]);
        }
        Stack.subVec(1);
    }

    @Override
    public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d supportVertexOut) {
        localGetSupportingVertexWithoutMargin(dir, supportVertexOut);
        if (getMargin() != 0.0) {
            Vector3d norm = Stack.newVec(dir);
            if (norm.lengthSquared() < (BulletGlobals.SIMD_EPSILON * BulletGlobals.SIMD_EPSILON)) {
                norm.set(-1.0, -1.0, -1.0);
            }
            norm.normalize();
            supportVertexOut.scaleAdd(getMargin(), norm, supportVertexOut);
            Stack.subVec(1);
        }
        return supportVertexOut;
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.CYLINDER_SHAPE_PROXYTYPE;
    }

    @SuppressWarnings("unused")
    public int getUpAxis() {
        return upAxis;
    }

    public double getRadius() {
        Vector3d tmp = Stack.newVec();
        double r = getHalfExtentsWithMargin(tmp).x;
        Stack.subVec(1);
        return r;
    }

    @Override
    public String getName() {
        return "CylinderY";
    }

}
