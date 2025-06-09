
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

    //protected final BulletStack stack = BulletStack.get();

    public double loLimit; //!< joint limit
    public double hiLimit; //!< joint limit
    public double targetVelocity; //!< target motor velocity
    public double maxMotorForce; //!< max force on motor
    public double maxLimitForce; //!< max force on limit
    public double damping; //!< Damping.
    public double limitSoftness; //! Relaxation factor
    public double ERP; //!< Error tolerance factor when joint is at limit
    public double bounce; //!< restitution factor
    public boolean enableMotor;

    public double currentLimitError;//!  How much is violated this limit
    public int currentLimit;//!< 0=free, 1=at lo limit, 2=at hi limit
    public double accumulatedImpulse;

    public RotationalLimitMotor() {
        accumulatedImpulse = 0.0;
        targetVelocity = 0;
        maxMotorForce = 0.1;
        maxLimitForce = 300.0;
        loLimit = -BulletGlobals.SIMD_INFINITY;
        hiLimit = BulletGlobals.SIMD_INFINITY;
        ERP = 0.5;
        bounce = 0.0;
        damping = 1.0;
        limitSoftness = 0.5;
        currentLimit = 0;
        currentLimitError = 0;
        enableMotor = false;
    }

    public RotationalLimitMotor(RotationalLimitMotor limot) {
        targetVelocity = limot.targetVelocity;
        maxMotorForce = limot.maxMotorForce;
        limitSoftness = limot.limitSoftness;
        loLimit = limot.loLimit;
        hiLimit = limot.hiLimit;
        ERP = limot.ERP;
        bounce = limot.bounce;
        currentLimit = limot.currentLimit;
        currentLimitError = limot.currentLimitError;
        enableMotor = limot.enableMotor;
    }

    /**
     * Is limited?
     */
    public boolean isLimited() {
        return !(loLimit >= hiLimit);
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
        if (loLimit > hiLimit) {
            currentLimit = 0; // Free from violation
            return 0;
        }

        if (test_value < loLimit) {
            currentLimit = 1; // low limit violation
            currentLimitError = test_value - loLimit;
            return 1;
        } else if (test_value > hiLimit) {
            currentLimit = 2; // High limit violation
            currentLimitError = test_value - hiLimit;
            return 2;
        }

        currentLimit = 0; // Free from violation
        return 0;
    }

    /**
     * Apply the correction impulses for two bodies.
     */
    @SuppressWarnings("UnusedReturnValue")
    public double solveAngularLimits(double timeStep, Vector3d axis, double jacDiagABInv, RigidBody body0, RigidBody body1) {
        if (!needApplyTorques()) {
            return 0.0;
        }

        double target_velocity = this.targetVelocity;
        double maxMotorForce = this.maxMotorForce;

        // current error correction
        if (currentLimit != 0) {
            target_velocity = -ERP * currentLimitError / (timeStep);
            maxMotorForce = maxLimitForce;
        }

        maxMotorForce *= timeStep;

        // current velocity difference
        Vector3d vel_diff = body0.getAngularVelocity(Stack.newVec());
        if (body1 != null) {
            vel_diff.sub(body1.getAngularVelocity(Stack.newVec()));
        }

        double rel_vel = axis.dot(vel_diff);

        // correction velocity
        double motor_relvel = limitSoftness * (target_velocity - damping * rel_vel);

        if (motor_relvel < BulletGlobals.FLT_EPSILON && motor_relvel > -BulletGlobals.FLT_EPSILON) {
            return 0.0; // no need for applying force
        }

        // correction impulse
        double unclippedMotorImpulse = (1 + bounce) * motor_relvel * jacDiagABInv;

        // clip correction impulse
        double clippedMotorImpulse;

        if (unclippedMotorImpulse > 0.0) {
            clippedMotorImpulse = Math.min(unclippedMotorImpulse, maxMotorForce);
        } else {
            clippedMotorImpulse = Math.max(unclippedMotorImpulse, -maxMotorForce);
        }

        // sort with accumulated impulses
        double lo = -1e30;
        double hi = 1e30;

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

        return clippedMotorImpulse;
    }

}
