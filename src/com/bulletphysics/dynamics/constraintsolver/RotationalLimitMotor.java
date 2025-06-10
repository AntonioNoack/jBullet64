
/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le�n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/
package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * Rotation limit structure for generic joints.
 *
 * @author jezek2
 */
public class RotationalLimitMotor {

    public double lowLimit;
    public double highLimit;
    public double targetVelocity;
    public double maxMotorForce;
    public double maxLimitForce;
    public double damping;

    /**
     * Relaxation factor
     * */
    public double limitSoftness;

    /**
     * Error tolerance factor when joint is at limit
     * */
    public double ERP;

    /**
     * restitution factor
     * */
    public double bounce;
    public boolean enableMotor;

    /**
     * How much is violated this limit
     * */
    public double currentLimitError;

    /**
     * 0=free, 1=at low limit, 2=at high limit
     * */
    public int currentLimit;
    public double accumulatedImpulse;

    public RotationalLimitMotor() {
        accumulatedImpulse = 0.0;
        targetVelocity = 0;
        maxMotorForce = 0.1;
        maxLimitForce = 300.0;
        lowLimit = -BulletGlobals.SIMD_INFINITY;
        highLimit = BulletGlobals.SIMD_INFINITY;
        ERP = 0.5;
        bounce = 0.0;
        damping = 1.0;
        limitSoftness = 0.5;
        currentLimit = 0;
        currentLimitError = 0;
        enableMotor = false;
    }

    /**
     * Is limited?
     */
    public boolean isLimited() {
        return !(lowLimit >= highLimit);
    }

    /**
     * Need apply correction?
     */
    public boolean needApplyTorques() {
        return currentLimit != 0 || enableMotor;
    }

    /**
     * Calculates error. Calculates currentLimit and currentLimitError.
     */
    @SuppressWarnings("UnusedReturnValue")
    public int testLimitValue(double test_value) {
        if (lowLimit > highLimit) {
            currentLimit = 0; // Free from violation
            return 0;
        }

        if (test_value < lowLimit) {
            currentLimit = 1; // low limit violation
            currentLimitError = test_value - lowLimit;
            return 1;
        } else if (test_value > highLimit) {
            currentLimit = 2; // High limit violation
            currentLimitError = test_value - highLimit;
            return 2;
        }

        currentLimit = 0; // Free from violation
        return 0;
    }

    /**
     * Apply the correction impulses for two bodies.
     */
    @SuppressWarnings("UnusedReturnValue")
    public double solveAngularLimits(
            double timeStep, Vector3d axis, double jacDiagABInv,
            RigidBody body0, RigidBody body1, TypedConstraint constraint
    ) {
        if (!needApplyTorques()) {
            return 0.0;
        }

        double targetVelocity = this.targetVelocity;
        double maxMotorForce = this.maxMotorForce;

        // current error correction
        if (currentLimit != 0) {
            targetVelocity = -ERP * currentLimitError / (timeStep);
            maxMotorForce = maxLimitForce;
        }

        maxMotorForce *= timeStep;

        // current velocity difference
        Vector3d velocityDifference = body0.getAngularVelocity(Stack.newVec());
        if (body1 != null) {
            velocityDifference.sub(body1.getAngularVelocity(Stack.newVec()));
        }

        double relativeVelocity = axis.dot(velocityDifference);

        // correction velocity
        double motorRelativeVelocity = limitSoftness * (targetVelocity - damping * relativeVelocity);

        if (motorRelativeVelocity < BulletGlobals.FLT_EPSILON && motorRelativeVelocity > -BulletGlobals.FLT_EPSILON) {
            Stack.subVec(2);
            return 0.0; // no need for applying force
        }

        // correction impulse
        double unclippedMotorImpulse = (1 + bounce) * motorRelativeVelocity * jacDiagABInv;
        if (Math.abs(unclippedMotorImpulse) > constraint.breakingImpulseThreshold) {
            constraint.setBroken(true);
            Stack.subVec(2);
            return 0.0;
        }

        // clip correction impulse
        double clippedMotorImpulse;

        if (unclippedMotorImpulse > 0.0) {
            clippedMotorImpulse = Math.min(unclippedMotorImpulse, maxMotorForce);
        } else {
            clippedMotorImpulse = Math.max(unclippedMotorImpulse, -maxMotorForce);
        }

        // sort with accumulated impulses
        double lo = -1e308;
        double hi = 1e308;

        double oldImpulseSum = accumulatedImpulse;
        double sum = oldImpulseSum + clippedMotorImpulse;
        accumulatedImpulse = sum > hi ? 0.0 : sum < lo ? 0.0 : sum;

        clippedMotorImpulse = accumulatedImpulse - oldImpulseSum;

        Vector3d motorImp = Stack.newVec();
        motorImp.scale(clippedMotorImpulse, axis);

        body0.applyTorqueImpulse(motorImp);
        if (body1 != null) {
            motorImp.negate();
            body1.applyTorqueImpulse(motorImp);
        }

        Stack.subVec(3);
        return clippedMotorImpulse;
    }

}
