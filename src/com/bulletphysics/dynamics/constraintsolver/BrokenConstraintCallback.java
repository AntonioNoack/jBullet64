package com.bulletphysics.dynamics.constraintsolver;

/**
 * Callback called for when constraints break.
 */
public interface BrokenConstraintCallback {
    void onBrokenConstraint(TypedConstraint constraint);
}
