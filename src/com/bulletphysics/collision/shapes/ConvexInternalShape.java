package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
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

        // all six main directions for the AABB
        // works, because the shape is convex, and centered around the origin

        Matrix3d basis = trans.basis;

        vec.set(1.0, 0.0, 0.0);
        maxAabb.x = transformGetSupportX(tmp1, tmp2, vec, basis);

        vec.x = -1.0;
        minAabb.x = transformGetSupportX(tmp1, tmp2, vec, basis);

        vec.x = 0.0;
        vec.y = 1.0;
        maxAabb.y = transformGetSupportY(tmp1, tmp2, vec, basis);

        vec.y = -1.0;
        minAabb.y = transformGetSupportY(tmp1, tmp2, vec, basis);

        vec.y = 0.0;
        vec.z = 1.0;
        maxAabb.z = transformGetSupportZ(tmp1, tmp2, vec, basis);

        vec.z = -1.0;
        minAabb.z = transformGetSupportZ(tmp1, tmp2, vec, basis);

        minAabb.add(trans.origin);
        minAabb.x -= margin;
        minAabb.y -= margin;
        minAabb.z -= margin;

        maxAabb.add(trans.origin);
        maxAabb.x += margin;
        maxAabb.y += margin;
        maxAabb.z += margin;

        Stack.subVec(3);

    }

    private double transformGetSupportX(Vector3d tmp1, Vector3d var1, Vector3d vec, Matrix3d trans) {
        MatrixUtil.transposeTransform(tmp1, vec, trans);// tmp1 = vec * transpose(trans.basis)
        localGetSupportingVertex(tmp1, var1);// tmp2 = getSupportInDirection(tmp1)
        return MatrixUtil.dotX(trans, var1);
    }

    private double transformGetSupportY(Vector3d tmp1, Vector3d var1, Vector3d vec, Matrix3d trans) {
        MatrixUtil.transposeTransform(tmp1, vec, trans);// tmp1 = vec * transpose(trans.basis)
        localGetSupportingVertex(tmp1, var1);// tmp2 = getSupportInDirection(tmp1)
        return MatrixUtil.dotY(trans, var1);
    }

    private double transformGetSupportZ(Vector3d tmp1, Vector3d var1, Vector3d vec, Matrix3d trans) {
        MatrixUtil.transposeTransform(tmp1, vec, trans);// tmp1 = vec * transpose(trans.basis)
        localGetSupportingVertex(tmp1, var1);// tmp2 = getSupportInDirection(tmp1)
        return  MatrixUtil.dotZ(trans, var1);
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

    public void setLocalScaling(Vector3d scaling) {
        localScaling.absolute(scaling);
    }

    public Vector3d getLocalScaling(Vector3d out) {
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
