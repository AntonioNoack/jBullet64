package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ConeShape implements a cone shape primitive, centered around the origin and
 * aligned with the Y axis. The {@link ConeShapeX} is aligned around the X axis
 * and {@link ConeShapeZ} around the Z axis.
 *
 * @author jezek2
 */
public class ConeShape extends ConvexInternalShape {

    private final double sinAngle;
    private final double radius;
    private final double height;
    private final int[] coneIndices = new int[3];

    public ConeShape(double radius, double height) {
        this.radius = radius;
        this.height = height;
        setConeUpIndex(1);
        sinAngle = (radius / Math.sqrt(this.radius * this.radius + this.height * this.height));
    }

    @SuppressWarnings("unused")
    public double getRadius() {
        return radius;
    }

    @SuppressWarnings("unused")
    public double getHeight() {
        return height;
    }

    private Vector3d coneLocalSupport(Vector3d v, Vector3d out) {
        double halfHeight = height * 0.5;
        if (VectorUtil.getCoord(v, coneIndices[1]) > v.length() * sinAngle) {
            VectorUtil.setCoord(out, coneIndices[0], 0.0);
            VectorUtil.setCoord(out, coneIndices[1], halfHeight);
            VectorUtil.setCoord(out, coneIndices[2], 0.0);
        } else {
            double v0 = VectorUtil.getCoord(v, coneIndices[0]);
            double v2 = VectorUtil.getCoord(v, coneIndices[2]);
            double s = Math.sqrt(v0 * v0 + v2 * v2);
            if (s > BulletGlobals.FLT_EPSILON) {
                double d = radius / s;
                VectorUtil.setCoord(out, coneIndices[0], VectorUtil.getCoord(v, coneIndices[0]) * d);
                VectorUtil.setCoord(out, coneIndices[1], -halfHeight);
                VectorUtil.setCoord(out, coneIndices[2], VectorUtil.getCoord(v, coneIndices[2]) * d);
            } else {
                VectorUtil.setCoord(out, coneIndices[0], 0.0);
                VectorUtil.setCoord(out, coneIndices[1], -halfHeight);
                VectorUtil.setCoord(out, coneIndices[2], 0.0);
            }
        }
        return out;
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        return coneLocalSupport(dir, out);
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] dirs, Vector3d[] outs, int numVectors) {
        for (int i = 0; i < numVectors; i++) {
            Vector3d vec = dirs[i];
            coneLocalSupport(vec, outs[i]);
        }
    }

    @Override
    public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out) {
        Vector3d supVertex = coneLocalSupport(dir, out);
        if (getMargin() != 0.0) {
            Vector3d vecNorm = Stack.newVec(dir);
            if (vecNorm.lengthSquared() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
                vecNorm.set(-1.0, -1.0, -1.0);
            }
            vecNorm.normalize();
            supVertex.scaleAdd(getMargin(), vecNorm, supVertex);
            Stack.subVec(1);
        }
        return supVertex;
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.CONE_SHAPE_PROXYTYPE;
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        Transform identity = Stack.newTrans();
        identity.setIdentity();
        Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
        getAabb(identity, aabbMin, aabbMax);

        Vector3d halfExtents = Stack.newVec();
        halfExtents.sub(aabbMax, aabbMin);
        halfExtents.scale(0.5);

        double margin = getMargin();

        double lx = 2.0 * (halfExtents.x + margin);
        double ly = 2.0 * (halfExtents.y + margin);
        double lz = 2.0 * (halfExtents.z + margin);
        double x2 = lx * lx;
        double y2 = ly * ly;
        double z2 = lz * lz;
        double scaledmass = mass * 0.08333333f;

        inertia.set(y2 + z2, x2 + z2, x2 + y2);
        inertia.scale(scaledmass);

        Stack.subVec(3);
        Stack.subTrans(1);
    }

    // choose upAxis index
    protected void setConeUpIndex(int upIndex) {
        switch (upIndex) {
            case 0:
                coneIndices[0] = 1;
                coneIndices[1] = 0;
                coneIndices[2] = 2;
                break;

            case 1:
                coneIndices[0] = 0;
                coneIndices[1] = 1;
                coneIndices[2] = 2;
                break;

            default:
                coneIndices[0] = 0;
                coneIndices[1] = 2;
                coneIndices[2] = 1;
                break;
        }
    }

    @SuppressWarnings("unused")
    public int getConeUpIndex() {
        return coneIndices[1];
    }

}
