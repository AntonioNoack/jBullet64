package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import org.jetbrains.annotations.NotNull;

import javax.vecmath.Vector3d;

/**
 * ConvexInternalShape is an internal base class, shared by most convex shape implementations.
 *
 * @author jezek2
 */
public abstract class ConvexInternalShape extends ConvexShape {

    // local scaling. collisionMargin is not scaled !
    protected final Vector3d localScaling = new Vector3d(1.0, 1.0, 1.0);
    protected final Vector3d implicitShapeDimensions = new Vector3d();
    protected double collisionMargin = BulletGlobals.CONVEX_DISTANCE_MARGIN;

    /**
     * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
     */
    @Override
    public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        getAabbSlow(t, aabbMin, aabbMax);
    }

    @Override
    public void getAabbSlow(Transform trans, Vector3d minAabb, Vector3d maxAabb) {
        double margin = getMargin();
        Vector3d vec = Stack.newVec();
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        for (int i = 0; i < 3; i++) {
            vec.set(0.0, 0.0, 0.0);
            VectorUtil.setCoord(vec, i, 1.0);

            MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
            localGetSupportingVertex(tmp1, tmp2);

            trans.transform(tmp2);

            VectorUtil.setCoord(maxAabb, i, VectorUtil.getCoord(tmp2, i) + margin);

            VectorUtil.setCoord(vec, i, -1.0);

            MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
            localGetSupportingVertex(tmp1, tmp2);
            trans.transform(tmp2);

            VectorUtil.setCoord(minAabb, i, VectorUtil.getCoord(tmp2, i) - margin);
        }
        Stack.subVec(3);
    }

    @Override
    public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out) {
        Vector3d supVertex = localGetSupportingVertexWithoutMargin(dir, out);

        if (getMargin() != 0.0) {
            Vector3d vecNorm = Stack.newVec(dir);
            if (vecNorm.lengthSquared() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
                vecNorm.set(-1.0, -1.0, -1.0);
            }
            vecNorm.normalize();
            supVertex.scaleAdd(getMargin(), vecNorm, supVertex);
            Stack.subVec(1);
        }
        return out;
    }

    public void setLocalScaling(@NotNull Vector3d scaling) {
        localScaling.absolute(scaling);
    }

    @NotNull
    public Vector3d getLocalScaling(@NotNull Vector3d out) {
        out.set(localScaling);
        return out;
    }

    public double getMargin() {
        return collisionMargin;
    }

    public void setMargin(double margin) {
        this.collisionMargin = margin;
    }

    @Override
    public int getNumPreferredPenetrationDirections() {
        return 0;
    }

    @Override
    public void getPreferredPenetrationDirection(int index, Vector3d penetrationVector) {
        throw new InternalError();
    }

}
