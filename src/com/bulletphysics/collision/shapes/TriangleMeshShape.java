package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import org.jetbrains.annotations.NotNull;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Concave triangle mesh abstract class. Use {@link BvhTriangleMeshShape} as concreteimplementation.
 *
 * @author jezek2
 */
public abstract class TriangleMeshShape extends ConcaveShape {

    protected final Vector3d localAabbMin = new Vector3d();
    protected final Vector3d localAabbMax = new Vector3d();
    protected final StridingMeshInterface meshInterface;

    protected TriangleMeshShape(StridingMeshInterface meshInterface) {
        this.meshInterface = meshInterface;
    }

    public Vector3d localGetSupportingVertex(Vector3d vec, Vector3d out) {

        Transform identity = Stack.newTrans();
        identity.setIdentity();
        SupportVertexCallback supportCallback = new SupportVertexCallback(vec, identity);

        Vector3d aabbMin = Stack.newVec();
        Vector3d aabbMax = Stack.newVec();
        aabbMax.set(1e308, 1e308, 1e308);
        aabbMin.set(-1e308, -1e308, -1e308);

        processAllTriangles(supportCallback, aabbMin, aabbMax);
        supportCallback.getSupportVertexLocal(out);

        Stack.subVec(2);
        Stack.subTrans(1);

        return out;
    }

    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d vec, Vector3d out) {
        assert (false);
        return localGetSupportingVertex(vec, out);
    }

    public void recalculateLocalAabb() {
        for (int i = 0; i < 3; i++) {
            Vector3d vec = Stack.newVec();
            vec.set(0.0, 0.0, 0.0);
            VectorUtil.setCoord(vec, i, 1.0);
            Vector3d tmp = localGetSupportingVertex(vec, Stack.newVec());
            VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmp, i) + getMargin());
            VectorUtil.setCoord(vec, i, -1.0);
            localGetSupportingVertex(vec, tmp);
            VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmp, i) - getMargin());
        }
    }

    @Override
    public void getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
        Vector3d tmp = Stack.newVec();

        Vector3d localHalfExtents = Stack.newVec();
        localHalfExtents.sub(localAabbMax, localAabbMin);
        localHalfExtents.scale(0.5);

        Vector3d localCenter = Stack.newVec();
        localCenter.add(localAabbMax, localAabbMin);
        localCenter.scale(0.5);

        Matrix3d abs_b = Stack.newMat(trans.basis);
        MatrixUtil.absolute(abs_b);

        Vector3d center = Stack.newVec(localCenter);
        trans.transform(center);

        Vector3d extent = Stack.newVec();
        abs_b.getRow(0, tmp);
        extent.x = tmp.dot(localHalfExtents);
        abs_b.getRow(1, tmp);
        extent.y = tmp.dot(localHalfExtents);
        abs_b.getRow(2, tmp);
        extent.z = tmp.dot(localHalfExtents);

        Vector3d margin = Stack.newVec();
        margin.set(getMargin(), getMargin(), getMargin());
        extent.add(margin);

        aabbMin.sub(center, extent);
        aabbMax.add(center, extent);
    }

    @Override
    public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
        FilteredCallback filterCallback = new FilteredCallback(callback, aabbMin, aabbMax);

        meshInterface.internalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        // moving concave objects not supported
        assert (false);
        inertia.set(0.0, 0.0, 0.0);
    }


    @Override
    public void setLocalScaling(Vector3d scaling) {
        meshInterface.setScaling(scaling);
        recalculateLocalAabb();
    }

    @Override
    public Vector3d getLocalScaling(Vector3d out) {
        return meshInterface.getScaling(out);
    }

    public StridingMeshInterface getMeshInterface() {
        return meshInterface;
    }

    public Vector3d getLocalAabbMin(Vector3d out) {
        out.set(localAabbMin);
        return out;
    }

    public Vector3d getLocalAabbMax(Vector3d out) {
        out.set(localAabbMax);
        return out;
    }

    ////////////////////////////////////////////////////////////////////////////

    private static class SupportVertexCallback implements TriangleCallback {
        private final Vector3d supportVertexLocal = new Vector3d(0.0, 0.0, 0.0);
        public final Transform worldTrans = new Transform();
        public double maxDot = -1e308;
        public final Vector3d supportVecLocal = new Vector3d();

        public SupportVertexCallback(Vector3d supportVecWorld, Transform trans) {
            this.worldTrans.set(trans);
            MatrixUtil.transposeTransform(supportVecLocal, supportVecWorld, worldTrans.basis);
        }

        public void processTriangle(@NotNull Vector3d[] triangle, int partId, int triangleIndex) {
            for (int i = 0; i < 3; i++) {
                double dot = supportVecLocal.dot(triangle[i]);
                if (dot > maxDot) {
                    maxDot = dot;
                    supportVertexLocal.set(triangle[i]);
                }
            }
        }

        public Vector3d getSupportVertexWorldSpace(Vector3d out) {
            out.set(supportVertexLocal);
            worldTrans.transform(out);
            return out;
        }

        @SuppressWarnings("UnusedReturnValue")
        public Vector3d getSupportVertexLocal(Vector3d out) {
            out.set(supportVertexLocal);
            return out;
        }
    }

    private static class FilteredCallback implements InternalTriangleIndexCallback {
        public TriangleCallback callback;
        public final Vector3d aabbMin = new Vector3d();
        public final Vector3d aabbMax = new Vector3d();

        public FilteredCallback(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
            this.callback = callback;
            this.aabbMin.set(aabbMin);
            this.aabbMax.set(aabbMax);
        }

        public void internalProcessTriangleIndex(Vector3d[] triangle, int partId, int triangleIndex) {
            if (AabbUtil2.testTriangleAgainstAabb2(triangle, aabbMin, aabbMax)) {
                // check aabb in triangle-space, before doing this
                callback.processTriangle(triangle, partId, triangleIndex);
            }
        }
    }

}
