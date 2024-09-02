package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;

import javax.vecmath.Vector3d;

/**
 * TypedConstraint is the base class for Bullet constraints and vehicles.
 *
 * @author jezek2
 */
public abstract class TypedConstraint {

    private static RigidBody sFixed;

    private static synchronized RigidBody getFixed() {
        if (sFixed == null) {
            sFixed = new RigidBody(0, null, null);
        }
        return sFixed;
    }

    private int userConstraintType = -1;
    private int userConstraintId = -1;

    private final TypedConstraintType constraintType;

    protected RigidBody rbA;
    protected RigidBody rbB;
    protected double appliedImpulse = 0.0;

    public TypedConstraint(TypedConstraintType type) {
        this(type, getFixed(), getFixed());
    }

    public TypedConstraint(TypedConstraintType type, RigidBody rbA) {
        this(type, rbA, getFixed());
    }

    public TypedConstraint(TypedConstraintType type, RigidBody rbA, RigidBody rbB) {
        this.constraintType = type;
        this.rbA = rbA;
        this.rbB = rbB;
        getFixed().setMassProps(0.0, new Vector3d(0.0, 0.0, 0.0));
    }

    public abstract void buildJacobian();

    public abstract void solveConstraint(double timeStep);

    public RigidBody getRigidBodyA() {
        return rbA;
    }

    public RigidBody getRigidBodyB() {
        return rbB;
    }

    public int getUserConstraintType() {
        return userConstraintType;
    }

    public void setUserConstraintType(int userConstraintType) {
        this.userConstraintType = userConstraintType;
    }

    public int getUserConstraintId() {
        return userConstraintId;
    }

    public int getUid() {
        return userConstraintId;
    }

    public void setUserConstraintId(int userConstraintId) {
        this.userConstraintId = userConstraintId;
    }

    public double getAppliedImpulse() {
        return appliedImpulse;
    }

    public TypedConstraintType getConstraintType() {
        return constraintType;
    }

}
