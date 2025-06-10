package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;

import javax.vecmath.Vector3d;

/**
 * TypedConstraint is the base class for Bullet constraints and vehicles.
 *
 * @author jezek2
 */
public abstract class TypedConstraint {

    private static final RigidBody FIXED = new RigidBody(0, null, null);

    public int userConstraintType = -1;
    public int userConstraintId = -1;

    public RigidBody rigidBodyA;
    public RigidBody rigidBodyB;
    public double appliedImpulse = 0.0;
    public double breakingImpulseThreshold = 1e308;

    public TypedConstraint() {
        this(FIXED, FIXED);
    }

    public TypedConstraint(RigidBody rigidBodyA) {
        this(rigidBodyA, FIXED);
    }

    public TypedConstraint(RigidBody rigidBodyA, RigidBody rigidBodyB) {
        this.rigidBodyA = rigidBodyA;
        this.rigidBodyB = rigidBodyB;
        FIXED.setMassProps(0.0, new Vector3d(0.0, 0.0, 0.0));
    }

    public abstract void buildJacobian();

    public abstract void solveConstraint(double timeStep);

    public boolean isBroken() {
        return breakingImpulseThreshold < 0;
    }

    public void setBroken(boolean broken) {
        if (broken != isBroken()) {
            breakingImpulseThreshold = -breakingImpulseThreshold;
        }
    }

}
