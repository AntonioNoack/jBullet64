package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.ScalarUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * BoxShape is a box primitive around the origin, its sides axis aligned with length
 * specified by half extents, in local shape coordinates. When used as part of a
 * {@link CollisionObject} or {@link RigidBody} it will be an oriented box in world space.
 *
 * @author jezek2
 */
public class BoxShape extends PolyhedralConvexShape {

    public BoxShape(Vector3d boxHalfExtents) {
        Vector3d margin = new Vector3d(getMargin(), getMargin(), getMargin());
        VectorUtil.mul(implicitShapeDimensions, boxHalfExtents, localScaling);
        implicitShapeDimensions.sub(margin);
    }

    public Vector3d getHalfExtentsWithMargin(Vector3d out) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(out);
        Vector3d margin = Stack.borrowVec();
        margin.set(getMargin(), getMargin(), getMargin());
        halfExtents.add(margin);
        return out;
    }

    public Vector3d getHalfExtentsWithoutMargin(Vector3d out) {
        out.set(implicitShapeDimensions); // changed in Bullet 2.63: assume the scaling and margin are included
        return out;
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.BOX_SHAPE_PROXYTYPE;
    }

    @Override
    public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(out);

        double margin = getMargin();
        halfExtents.x += margin;
        halfExtents.y += margin;
        halfExtents.z += margin;

        out.set(
                ScalarUtil.select(dir.x, halfExtents.x, -halfExtents.x),
                ScalarUtil.select(dir.y, halfExtents.y, -halfExtents.y),
                ScalarUtil.select(dir.z, halfExtents.z, -halfExtents.z));
        return out;
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(out);
        out.set(
                ScalarUtil.select(dir.x, halfExtents.x, -halfExtents.x),
                ScalarUtil.select(dir.y, halfExtents.y, -halfExtents.y),
                ScalarUtil.select(dir.z, halfExtents.z, -halfExtents.z));
        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(Stack.newVec());

        for (int i = 0; i < numVectors; i++) {
            Vector3d vec = vectors[i];
            supportVerticesOut[i].set(ScalarUtil.select(vec.x, halfExtents.x, -halfExtents.x),
                    ScalarUtil.select(vec.y, halfExtents.y, -halfExtents.y),
                    ScalarUtil.select(vec.z, halfExtents.z, -halfExtents.z));
        }
    }

    @Override
    public void setMargin(double margin) {
        // correct the implicitShapeDimensions for the margin
        Vector3d oldMargin = Stack.newVec();
        oldMargin.set(getMargin(), getMargin(), getMargin());
        Vector3d implicitShapeDimensionsWithMargin = Stack.newVec();
        implicitShapeDimensionsWithMargin.add(implicitShapeDimensions, oldMargin);

        super.setMargin(margin);
        Vector3d newMargin = Stack.newVec();
        newMargin.set(getMargin(), getMargin(), getMargin());
        implicitShapeDimensions.sub(implicitShapeDimensionsWithMargin, newMargin);
        Stack.subVec(3);
    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        Vector3d oldMargin = Stack.newVec();
        oldMargin.set(getMargin(), getMargin(), getMargin());
        Vector3d implicitShapeDimensionsWithMargin = Stack.newVec();
        implicitShapeDimensionsWithMargin.add(implicitShapeDimensions, oldMargin);
        Vector3d unScaledImplicitShapeDimensionsWithMargin = Stack.newVec();
        VectorUtil.div(unScaledImplicitShapeDimensionsWithMargin, implicitShapeDimensionsWithMargin, localScaling);

        super.setLocalScaling(scaling);

        VectorUtil.mul(implicitShapeDimensions, unScaledImplicitShapeDimensionsWithMargin, localScaling);
        implicitShapeDimensions.sub(oldMargin);
        Stack.subVec(3);
    }

    @Override
    public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        AabbUtil2.transformAabb(getHalfExtentsWithoutMargin(Stack.newVec()), getMargin(), t, aabbMin, aabbMax);
        Stack.subVec(1);
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        Vector3d halfExtents = getHalfExtentsWithMargin(Stack.newVec());

        double lx = 2.0 * halfExtents.x;
        double ly = 2.0 * halfExtents.y;
        double lz = 2.0 * halfExtents.z;

        inertia.set(mass / 12.0 * (ly * ly + lz * lz),
                mass / 12.0 * (lx * lx + lz * lz),
                mass / 12.0 * (lx * lx + ly * ly));

        Stack.subVec(1);
    }

    @Override
    public void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i) {
        // this plane might not be aligned...
        Vector4d plane = new Vector4d();
        getPlaneEquation(plane, i);
        planeNormal.set(plane.x, plane.y, plane.z);
        Vector3d tmp = Stack.newVec();
        tmp.negate(planeNormal);
        localGetSupportingVertex(tmp, planeSupport);
    }

    @Override
    public int getNumPlanes() {
        return 6;
    }

    @Override
    public int getNumVertices() {
        return 8;
    }

    @Override
    public int getNumEdges() {
        return 12;
    }

    @Override
    public void getVertex(int i, Vector3d vtx) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(Stack.newVec());

        vtx.set(halfExtents.x * (1 - (i & 1)) - halfExtents.x * (i & 1),
                halfExtents.y * (1 - ((i & 2) >> 1)) - halfExtents.y * ((i & 2) >> 1),
                halfExtents.z * (1 - ((i & 4) >> 2)) - halfExtents.z * ((i & 4) >> 2));
    }

    public void getPlaneEquation(Vector4d plane, int i) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(Stack.newVec());
        double axisValue = (i & 1) == 0 ? 1.0 : -1.0;
        int axis = i >> 1;
        switch (axis) {
            case 0:
                plane.set(axisValue, 0.0, 0.0, -halfExtents.x);
                break;
            case 1:
                plane.set(0.0, axisValue, 0.0, -halfExtents.y);
                break;
            default:
                plane.set(0.0, 0.0, axisValue, -halfExtents.z);
        }
    }

    @Override
    public void getEdge(int i, Vector3d pa, Vector3d pb) {
        int edgeVert0 = 0;
        int edgeVert1 = 0;
        switch (i) {
            case 0:
                edgeVert1 = 1;
                break;
            case 1:
                edgeVert1 = 2;
                break;
            case 2:
                edgeVert0 = 1;
                edgeVert1 = 3;
                break;
            case 3:
                edgeVert0 = 2;
                edgeVert1 = 3;
                break;
            case 4:
                edgeVert1 = 4;
                break;
            case 5:
                edgeVert0 = 1;
                edgeVert1 = 5;

                break;
            case 6:
                edgeVert0 = 2;
                edgeVert1 = 6;
                break;
            case 7:
                edgeVert0 = 3;
                edgeVert1 = 7;
                break;
            case 8:
                edgeVert0 = 4;
                edgeVert1 = 5;
                break;
            case 9:
                edgeVert0 = 4;
                edgeVert1 = 6;
                break;
            case 10:
                edgeVert0 = 5;
                edgeVert1 = 7;
                break;
            case 11:
                edgeVert0 = 6;
                edgeVert1 = 7;
                break;
            default:
                assert (false);
        }

        getVertex(edgeVert0, pa);
        getVertex(edgeVert1, pb);
    }

    @Override
    public boolean isInside(Vector3d pt, double tolerance) {
        Vector3d halfExtents = getHalfExtentsWithoutMargin(Stack.newVec());
        return (pt.x <= (halfExtents.x + tolerance)) &&
                (pt.x >= (-halfExtents.x - tolerance)) &&
                (pt.y <= (halfExtents.y + tolerance)) &&
                (pt.y >= (-halfExtents.y - tolerance)) &&
                (pt.z <= (halfExtents.z + tolerance)) &&
                (pt.z >= (-halfExtents.z - tolerance));
    }

    @Override
    public int getNumPreferredPenetrationDirections() {
        return 6;
    }

    @Override
    public void getPreferredPenetrationDirection(int index, Vector3d penetrationVector) {
        int axis = index >> 1;
        double value = (index & 1) == 0 ? 1.0 : -1.0;
        penetrationVector.set(0.0, 0.0, 0.0);
        VectorUtil.setCoord(penetrationVector, axis, value);
    }

}
