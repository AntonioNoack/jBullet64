package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import java.util.ArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

// JAVA NOTE: CompoundShape from 2.71

/**
 * CompoundShape allows to store multiple other {@link CollisionShape}s. This allows
 * for moving concave collision objects. This is more general than the {@link BvhTriangleMeshShape}.
 *
 * @author jezek2
 */
public class CompoundShape extends CollisionShape {

    private final ArrayList<CompoundShapeChild> children = new ArrayList<>();
    private final Vector3d localAabbMin = new Vector3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final Vector3d localAabbMax = new Vector3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

    private final OptimizedBvh aabbTree = null;

    private double collisionMargin = 0.0;
    protected final Vector3d localScaling = new Vector3d(1.0, 1.0, 1.0);

    public void addChildShape(Transform localTransform, CollisionShape shape) {
        //m_childTransforms.push_back(localTransform);
        //m_childShapes.push_back(shape);
        CompoundShapeChild child = new CompoundShapeChild();
        child.transform.set(localTransform);
        child.childShape = shape;
        child.childShapeType = shape.getShapeType();
        child.childMargin = shape.getMargin();

        children.add(child);

        // extend the local aabbMin/aabbMax
        Vector3d _localAabbMin = Stack.newVec(), _localAabbMax = Stack.newVec();
        shape.getAabb(localTransform, _localAabbMin, _localAabbMax);

        // JAVA NOTE: rewritten
//		for (int i=0;i<3;i++)
//		{
//			if (this.localAabbMin[i] > _localAabbMin[i])
//			{
//				this.localAabbMin[i] = _localAabbMin[i];
//			}
//			if (this.localAabbMax[i] < _localAabbMax[i])
//			{
//				this.localAabbMax[i] = _localAabbMax[i];
//			}
//		}
        VectorUtil.setMin(this.localAabbMin, _localAabbMin);
        VectorUtil.setMax(this.localAabbMax, _localAabbMax);

        Stack.subVec(2);

    }

    /**
     * Remove all children shapes that contain the specified shape.
     */
    public void removeChildShape(CollisionShape shape) {
        boolean done_removing;

        // Find the children containing the shape specified, and remove those children.
        do {
            done_removing = true;

            for (int i = 0; i < children.size(); i++) {
                if (children.get(i).childShape == shape) {
                    children.remove(i);
                    done_removing = false;  // Do another iteration pass after removing from the vector
                    break;
                }
            }
        }
        while (!done_removing);

        recalculateLocalAabb();
    }

    public int getNumChildShapes() {
        return children.size();
    }

    public CollisionShape getChildShape(int index) {
        return children.get(index).childShape;
    }

    public Transform getChildTransform(int index, Transform out) {
        out.set(children.get(index).transform);
        return out;
    }

    public ArrayList<CompoundShapeChild> getChildList() {
        return children;
    }

    /**
     * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
     */
    @Override
    public void getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
        Vector3d localHalfExtents = Stack.newVec();
        localHalfExtents.sub(localAabbMax, localAabbMin);
        localHalfExtents.scale(0.5);
        localHalfExtents.x += getMargin();
        localHalfExtents.y += getMargin();
        localHalfExtents.z += getMargin();

        Vector3d localCenter = Stack.newVec();
        localCenter.add(localAabbMax, localAabbMin);
        localCenter.scale(0.5);

        Matrix3d abs_b = Stack.newMat(trans.basis);
        MatrixUtil.absolute(abs_b);

        Vector3d center = Stack.newVec(localCenter);
        trans.transform(center);

        Vector3d tmp = Stack.newVec();

        Vector3d extent = Stack.newVec();
        abs_b.getRow(0, tmp);
        extent.x = tmp.dot(localHalfExtents);
        abs_b.getRow(1, tmp);
        extent.y = tmp.dot(localHalfExtents);
        abs_b.getRow(2, tmp);
        extent.z = tmp.dot(localHalfExtents);

        aabbMin.sub(center, extent);
        aabbMax.add(center, extent);
    }

    /**
     * Re-calculate the local Aabb. Is called at the end of removeChildShapes.
     * Use this yourself if you modify the children or their transforms.
     */
    public void recalculateLocalAabb() {
        // Recalculate the local aabb
        // Brute force, it iterates over all the shapes left.
        localAabbMin.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        localAabbMax.set(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

        Vector3d tmpLocalAabbMin = Stack.newVec();
        Vector3d tmpLocalAabbMax = Stack.newVec();

        // extend the local aabbMin/aabbMax
        for (int j = 0; j < children.size(); j++) {
            children.get(j).childShape.getAabb(children.get(j).transform, tmpLocalAabbMin, tmpLocalAabbMax);

            localAabbMin.x = Math.min(localAabbMin.x, tmpLocalAabbMin.x);
            localAabbMin.y = Math.min(localAabbMin.y, tmpLocalAabbMin.y);
            localAabbMin.z = Math.min(localAabbMin.z, tmpLocalAabbMin.z);

            localAabbMax.x = Math.max(localAabbMax.x, tmpLocalAabbMax.x);
            localAabbMax.y = Math.max(localAabbMax.y, tmpLocalAabbMax.y);
            localAabbMax.z = Math.max(localAabbMax.z, tmpLocalAabbMax.z);
        }
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
        // approximation: take the inertia from the aabb for now
        Transform identity = new Transform();
        identity.setIdentity();
        Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
        getAabb(identity, aabbMin, aabbMax);

        Vector3d halfExtents = Stack.newVec();
        halfExtents.sub(aabbMax, aabbMin);
        halfExtents.scale(0.5);

        double lx = halfExtents.x;
        double ly = halfExtents.y;
        double lz = halfExtents.z;

        double factor = mass / 3.0;

        double lx2 = lx * lx;
        double ly2 = ly * ly;
        double lz2 = lz * lz;
        inertia.x = factor * (ly2 + lz2);
        inertia.y = factor * (lx2 + lz2);
        inertia.z = factor * (lx2 + ly2);
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.COMPOUND_SHAPE_PROXYTYPE;
    }

    @Override
    public void setMargin(double margin) {
        collisionMargin = margin;
    }

    @Override
    public double getMargin() {
        return collisionMargin;
    }

    @Override
    public String getName() {
        return "Compound";
    }

    // this is optional, but should make collision queries faster, by culling non-overlapping nodes
    // void	createAabbTreeFromChildren();

    public OptimizedBvh getAabbTree() {
        return aabbTree;
    }

    /**
     * Computes the exact moment of inertia and the transform from the coordinate
     * system defined by the principal axes of the moment of inertia and the center
     * of mass to the current coordinate system. "masses" points to an array
     * of masses of the children. The resulting transform "principal" has to be
     * applied inversely to all children transforms in order for the local coordinate
     * system of the compound shape to be centered at the center of mass and to coincide
     * with the principal axes. This also necessitates a correction of the world transform
     * of the collision object by the principal transform.
     */
    public void calculatePrincipalAxisTransform(double[] masses, Transform principal, Vector3d inertia) {
        int n = children.size();

        double totalMass = 0;
        Vector3d center = Stack.newVec();
        center.set(0, 0, 0);
        for (int k = 0; k < n; k++) {
            center.scaleAdd(masses[k], children.get(k).transform.origin, center);
            totalMass += masses[k];
        }
        center.scale(1.0 / totalMass);
        principal.origin.set(center);

        Matrix3d tensor = Stack.newMat();
        tensor.setZero();

        for (int k = 0; k < n; k++) {
            Vector3d i = Stack.newVec();
            children.get(k).childShape.calculateLocalInertia(masses[k], i);

            Transform t = children.get(k).transform;
            Vector3d o = Stack.newVec();
            o.sub(t.origin, center);

            // compute inertia tensor in coordinate system of compound shape
            Matrix3d j = Stack.newMat();
            j.transpose(t.basis);

            j.m00 *= i.x;
            j.m01 *= i.x;
            j.m02 *= i.x;
            j.m10 *= i.y;
            j.m11 *= i.y;
            j.m12 *= i.y;
            j.m20 *= i.z;
            j.m21 *= i.z;
            j.m22 *= i.z;

            j.mul(t.basis, j);

            // add inertia tensor
            tensor.add(j);

            // compute inertia tensor of pointmass at o
            double o2 = o.lengthSquared();
            j.setRow(0, o2, 0, 0);
            j.setRow(1, 0, o2, 0);
            j.setRow(2, 0, 0, o2);
            j.m00 += o.x * -o.x;
            j.m01 += o.y * -o.x;
            j.m02 += o.z * -o.x;
            j.m10 += o.x * -o.y;
            j.m11 += o.y * -o.y;
            j.m12 += o.z * -o.y;
            j.m20 += o.x * -o.z;
            j.m21 += o.y * -o.z;
            j.m22 += o.z * -o.z;

            // add inertia tensor of pointmass
            tensor.m00 += masses[k] * j.m00;
            tensor.m01 += masses[k] * j.m01;
            tensor.m02 += masses[k] * j.m02;
            tensor.m10 += masses[k] * j.m10;
            tensor.m11 += masses[k] * j.m11;
            tensor.m12 += masses[k] * j.m12;
            tensor.m20 += masses[k] * j.m20;
            tensor.m21 += masses[k] * j.m21;
            tensor.m22 += masses[k] * j.m22;
        }

        MatrixUtil.diagonalize(tensor, principal.basis, 0.00001f, 20);

        inertia.set(tensor.m00, tensor.m11, tensor.m22);
    }

}
