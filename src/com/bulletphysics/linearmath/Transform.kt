package com.bulletphysics.linearmath;

import com.bulletphysics.collision.shapes.UniformScalingShape;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * Transform represents translation and rotation (rigid transform). Scaling and
 * shearing is not supported.<p>
 * <p>
 * You can use local shape scaling or {@link UniformScalingShape} for static rescaling
 * of collision objects.
 *
 * @author jezek2
 */
public class Transform {

    /**
     * Rotation matrix of this Transform.
     */
    public final Matrix3d basis = new Matrix3d();

    /**
     * Translation vector of this Transform.
     */
    public final Vector3d origin = new Vector3d();

    public Transform() {
    }

    public Transform(Matrix3d mat) {
        basis.set(mat);
    }

    public Transform(Matrix4d mat) {
        set(mat);
    }

    public Transform(Transform tr) {
        set(tr);
    }

    public void set(Transform tr) {
        basis.set(tr.basis);
        origin.set(tr.origin);
    }

    public void set(Matrix3d mat) {
        basis.set(mat);
        origin.set(0.0, 0.0, 0.0);
    }

    public void set(Matrix4d mat) {
        mat.getRotationScale(basis);
        origin.set(mat.m03, mat.m13, mat.m23);
    }

    public void transform(Vector3d v) {
        basis.transform(v);
        v.add(origin);
    }

    public void setIdentity() {
        basis.setIdentity();
        origin.set(0.0, 0.0, 0.0);
    }

    public void inverse() {
        basis.transpose();
        origin.scale(-1.0);
        basis.transform(origin);
    }

    public void inverse(Transform tr) {
        set(tr);
        inverse();
    }

    public void mul(Transform tr) {
        Vector3d vec = Stack.borrowVec(tr.origin);
        transform(vec);
        basis.mul(tr.basis);
        origin.set(vec);
    }

    public void mul(Transform tr1, Transform tr2) {
        Vector3d vec = Stack.borrowVec(tr2.origin);
        tr1.transform(vec);
        basis.mul(tr1.basis, tr2.basis);
        origin.set(vec);
    }

    public void invXform(Vector3d inVec, Vector3d out) {
        out.sub(inVec, origin);
        Matrix3d mat = Stack.borrowMat(basis);
        mat.transpose();
        mat.transform(out);
    }

    public Quat4d getRotation(Quat4d out) {
        MatrixUtil.getRotation(basis, out);
        return out;
    }

    public void setRotation(Quat4d q) {
        MatrixUtil.setRotation(basis, q);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Transform)) return false;
        Transform tr = (Transform) obj;
        return basis.equals(tr.basis) && origin.equals(tr.origin);
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 41 * hash + basis.hashCode();
        hash = 41 * hash + origin.hashCode();
        return hash;
    }

}
