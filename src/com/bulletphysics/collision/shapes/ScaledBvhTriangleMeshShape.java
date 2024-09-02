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
        invLocalScaling.set(1.0 / localScaling.x, 1.0 / localScaling.y, 1.0 / localScaling.z);

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
        return "SCALED_BVH_TRIANGLE_MESH";
    }

    ////////////////////////////////////////////////////////////////////////////

    private static class ScaledTriangleCallback extends TriangleCallback {
        private final TriangleCallback originalCallback;
        private final Vector3d localScaling;
        private final Vector3d[] newTriangle = new Vector3d[3];

        public ScaledTriangleCallback(TriangleCallback originalCallback, Vector3d localScaling) {
            this.originalCallback = originalCallback;
            this.localScaling = localScaling;

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
