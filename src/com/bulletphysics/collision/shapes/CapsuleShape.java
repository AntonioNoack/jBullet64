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
        implicitShapeDimensions.set(radius, 0.5 * height, radius);
    }

    public CapsuleShape(double radius, double height, int upAxis) {
        this.upAxis = upAxis;
        switch (upAxis) {
            case 0:
                implicitShapeDimensions.set(0.5 * height, radius, radius);
                break;
            case 1:
                implicitShapeDimensions.set(radius, 0.5 * height, radius);
                break;
            case 2:
                implicitShapeDimensions.set(radius, radius, 0.5 * height);
                break;
            default:
                throw new IllegalArgumentException("Axis must be 0-2");
        }
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d supVec) {
        supVec.set(0, 0, 0);

        double maxDot = Double.NEGATIVE_INFINITY;

        int v3 = Stack.getVecPosition();

        Vector3d vec = Stack.newVec(dir);
        VectorUtil.normalizeSafely(vec);

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

        double lx = halfExtents.x + margin;
        double ly = halfExtents.y + margin;
        double lz = halfExtents.z + margin;
        double x2 = lx * lx;
        double y2 = ly * ly;
        double z2 = lz * lz;

        inertia.set(y2 + z2, x2 + z2, x2 + y2);
        inertia.scale(mass / 3.0);
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
