package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * CollisionShape class provides an interface for collision shapes that can be
 * shared among {@link CollisionObject}s.
 *
 * @author jezek2
 */
public abstract class CollisionShape {

    protected Object userPointer;

    // getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
    public abstract void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax);

    public void getBoundingSphere(Vector3d center, double[] radius) {
        Vector3d tmp = Stack.newVec();

        Transform tr = Stack.borrowTrans();
        tr.setIdentity();
        Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();

        getAabb(tr, aabbMin, aabbMax);

        tmp.sub(aabbMax, aabbMin);
        radius[0] = tmp.length() * 0.5;

        tmp.add(aabbMin, aabbMax);
        center.scale(0.5, tmp);

        Stack.subVec(3);

    }

    // getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
    public double getAngularMotionDisc() {
        Vector3d center = Stack.newVec();
        double[] disc = new double[1]; // TODO: stack
        getBoundingSphere(center, disc);
        disc[0] += center.length();
        return disc[0];
    }

    // calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
    // result is conservative
    public void calculateTemporalAabb(Transform curTrans, Vector3d linvel, Vector3d angvel, double timeStep, Vector3d temporalAabbMin, Vector3d temporalAabbMax) {
        //start with static aabb
        getAabb(curTrans, temporalAabbMin, temporalAabbMax);

        double temporalAabbMaxx = temporalAabbMax.x;
        double temporalAabbMaxy = temporalAabbMax.y;
        double temporalAabbMaxz = temporalAabbMax.z;
        double temporalAabbMinx = temporalAabbMin.x;
        double temporalAabbMiny = temporalAabbMin.y;
        double temporalAabbMinz = temporalAabbMin.z;

        // add linear motion
        Vector3d linMotion = Stack.newVec(linvel);
        linMotion.scale(timeStep);

        temporalAabbMaxx += Math.max(linMotion.x, 0.0);
        temporalAabbMinx += Math.min(linMotion.x, 0.0);
        temporalAabbMaxy += Math.max(linMotion.y, 0.0);
        temporalAabbMiny += Math.min(linMotion.y, 0.0);
        temporalAabbMaxz += Math.max(linMotion.z, 0.0);
        temporalAabbMinz += Math.min(linMotion.z, 0.0);

        //add conservative angular motion
        double angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
        Vector3d angularMotion3d = Stack.borrowVec();
        angularMotion3d.set(angularMotion, angularMotion, angularMotion);
        temporalAabbMin.set(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
        temporalAabbMax.set(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

        temporalAabbMin.sub(angularMotion3d);
        temporalAabbMax.add(angularMotion3d);
    }

    //#ifndef __SPU__
    public boolean isPolyhedral() {
        return getShapeType().isPolyhedral();
    }

    public boolean isConvex() {
        return getShapeType().isConvex();
    }

    public boolean isConcave() {
        return getShapeType().isConcave();
    }

    public boolean isCompound() {
        return getShapeType().isCompound();
    }

    // isInfinite is used to catch simulation error (aabb check)
    public boolean isInfinite() {
        return getShapeType().isInfinite();
    }

    public abstract BroadphaseNativeType getShapeType();

    public abstract void setLocalScaling(Vector3d scaling);

    // TODO: returns const
    public abstract Vector3d getLocalScaling(Vector3d out);

    public abstract void calculateLocalInertia(double mass, Vector3d inertia);

    //debugging support
    public abstract String getName();

    //#endif //__SPU__
    public abstract void setMargin(double margin);

    public abstract double getMargin();

    // optional user data pointer
    public void setUserPointer(Object userPtr) {
        userPointer = userPtr;
    }

    public Object getUserPointer() {
        return userPointer;
    }

}
