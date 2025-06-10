package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;

import javax.vecmath.Vector3d;

/**
 * TypedConstraint is the base class for Bullet constraints and vehicles.
 *
 * @author jezek2
 */
public abstract class TypedConstraint {

    private static final RigidBody sFixed = new RigidBody(0, null, null);

    private static RigidBody getFixed() {
        return sFixed;
    }

    private int userConstraintType = -1;
    private int userConstraintId = -1;

    private final TypedConstraintType constraintType;

    protected RigidBody rbA;
    protected RigidBody rbB;
    protected double appliedImpulse = 0.0;
    protected double breakingImpulseThreshold = 1e308;

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

    @SuppressWarnings("unused")
    public int getUserConstraintType() {
        return userConstraintType;
    }

    @SuppressWarnings("unused")
    public void setUserConstraintType(int userConstraintType) {
        this.userConstraintType = userConstraintType;
    }

    @SuppressWarnings("unused")
    public int getUserConstraintId() {
        return userConstraintId;
    }

    public double getBreakingImpulseThreshold() {
        return breakingImpulseThreshold;
    }

    public void setBreakingImpulseThreshold(double breakingImpulseThreshold) {
        this.breakingImpulseThreshold = breakingImpulseThreshold;
    }

    @SuppressWarnings("unused")
    public boolean isBroken() {
        return breakingImpulseThreshold < 0;
    }

    @SuppressWarnings("unused")
    public void setBroken(boolean broken) {
        if (broken != isBroken()) {
            breakingImpulseThreshold = -breakingImpulseThreshold;
        }
    }

    @SuppressWarnings("unused")
    public int getUid() {
        return userConstraintId;
    }

    @SuppressWarnings("unused")
    public void setUserConstraintId(int userConstraintId) {
        this.userConstraintId = userConstraintId;
    }

    @SuppressWarnings("unused")
    public double getAppliedImpulse() {
        return appliedImpulse;
    }

    @SuppressWarnings("unused")
    public TypedConstraintType getConstraintType() {
        return constraintType;
    }

}
