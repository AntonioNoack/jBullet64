package com.bulletphysics.dynamics.constraintsolver;

import javax.vecmath.Vector3d;

/**
 * 1D constraint along a normal axis between bodyA and bodyB. It can be combined
 * to solve contact and friction constraints.
 *
 * @author jezek2
 */
public class SolverConstraint {

    public final Vector3d relPos1CrossNormal = new Vector3d();
    public final Vector3d contactNormal = new Vector3d();

    public final Vector3d relPos2CrossNormal = new Vector3d();
    public final Vector3d angularComponentA = new Vector3d();

    public final Vector3d angularComponentB = new Vector3d();

    public double appliedPushImpulse;

    public double appliedImpulse;
    public int solverBodyIdA;
    public int solverBodyIdB;

    public double friction;
    public double restitution;
    public double jacDiagABInv;
    public double penetration;

    public SolverConstraintType constraintType;
    public int frictionIndex;
    public Object originalContactPoint;

}
