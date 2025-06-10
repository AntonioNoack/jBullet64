package com.bulletphysics.dynamics.constraintsolver;

import javax.vecmath.Vector3d;

/**
 * Stores some extra information to each contact point. It is not in the contact
 * point, because that want to keep the collision detection independent from the
 * constraint solver.
 *
 * @author jezek2
 */
public class ConstraintPersistentData {

    /**
     * total applied impulse during most recent frame
     */
    public double appliedImpulse = 0.0;
    public double prevAppliedImpulse = 0.0;
    public double accumulatedTangentImpulse0 = 0.0;
    public double accumulatedTangentImpulse1 = 0.0;

    public double jacDiagABInv = 0.0;
    public double jacDiagABInvTangent0;
    public double jacDiagABInvTangent1;
    public int persistentLifeTime = 0;
    public double restitution = 0.0;
    public double friction = 0.0;
    public double penetration = 0.0;
    public final Vector3d frictionWorldTangential0 = new Vector3d();
    public final Vector3d frictionWorldTangential1 = new Vector3d();

    public final Vector3d frictionAngularComponent0A = new Vector3d();
    public final Vector3d frictionAngularComponent0B = new Vector3d();
    public final Vector3d frictionAngularComponent1A = new Vector3d();
    public final Vector3d frictionAngularComponent1B = new Vector3d();

    //some data doesn't need to be persistent over frames: todo: clean/reuse this
    public final Vector3d angularComponentA = new Vector3d();
    public final Vector3d angularComponentB = new Vector3d();

    public ContactSolverFunc contactSolverFunc = null;
    public ContactSolverFunc frictionSolverFunc = null;

    public void reset() {
        appliedImpulse = 0.0;
        prevAppliedImpulse = 0.0;
        accumulatedTangentImpulse0 = 0.0;
        accumulatedTangentImpulse1 = 0.0;

        jacDiagABInv = 0.0;
        jacDiagABInvTangent0 = 0.0;
        jacDiagABInvTangent1 = 0.0;
        persistentLifeTime = 0;
        restitution = 0.0;
        friction = 0.0;
        penetration = 0.0;
        frictionWorldTangential0.set(0.0, 0.0, 0.0);
        frictionWorldTangential1.set(0.0, 0.0, 0.0);

        frictionAngularComponent0A.set(0.0, 0.0, 0.0);
        frictionAngularComponent0B.set(0.0, 0.0, 0.0);
        frictionAngularComponent1A.set(0.0, 0.0, 0.0);
        frictionAngularComponent1B.set(0.0, 0.0, 0.0);

        angularComponentA.set(0.0, 0.0, 0.0);
        angularComponentB.set(0.0, 0.0, 0.0);

        contactSolverFunc = null;
        frictionSolverFunc = null;
    }

}
