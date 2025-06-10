
/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le�n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/
package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class TranslationalLimitMotor {

    //protected final BulletStack stack = BulletStack.get();

    public final Vector3d lowerLimit = new Vector3d(); //!< the constraint lower limits
    public final Vector3d upperLimit = new Vector3d(); //!< the constraint upper limits
    public final Vector3d accumulatedImpulse = new Vector3d();

    public double limitSoftness; //!< Softness for linear limit
    public double damping; //!< Damping for linear limit
    public double restitution; //! Bounce parameter for linear limit

    public TranslationalLimitMotor() {
        lowerLimit.set(0.0, 0.0, 0.0);
        upperLimit.set(0.0, 0.0, 0.0);
        accumulatedImpulse.set(0.0, 0.0, 0.0);

        limitSoftness = 0.7;
        damping = 1.0;
        restitution = 0.5;
    }

    public TranslationalLimitMotor(TranslationalLimitMotor other) {
        lowerLimit.set(other.lowerLimit);
        upperLimit.set(other.upperLimit);
        accumulatedImpulse.set(other.accumulatedImpulse);

        limitSoftness = other.limitSoftness;
        damping = other.damping;
        restitution = other.restitution;
    }

    /**
     * Test limit.<p>
     * - free means upper &lt; lower,<br>
     * - locked means upper == lower<br>
     * - limited means upper &gt; lower<br>
     * - limitIndex: first 3 are linear, next 3 are angular
     */
    public boolean isLimited(int limitIndex) {
        return (VectorUtil.getCoord(upperLimit, limitIndex) >= VectorUtil.getCoord(lowerLimit, limitIndex));
    }

    @SuppressWarnings("UnusedReturnValue")
    public double solveLinearAxis(
            double timeStep, double jacDiagABInv, RigidBody body1, Vector3d pointInA,
            RigidBody body2, Vector3d pointInB, int limit_index,
            Vector3d axisNormalOnA, Vector3d anchorPos
    ) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmpVec = Stack.newVec();

        // find relative velocity
        Vector3d relPos1 = Stack.newVec();
        //relPos1.sub(pointInA, body1.getCenterOfMassPosition(tmpVec));
        relPos1.sub(anchorPos, body1.getCenterOfMassPosition(tmpVec));

        Vector3d relPos2 = Stack.newVec();
        //relPos2.sub(pointInB, body2.getCenterOfMassPosition(tmpVec));
        relPos2.sub(anchorPos, body2.getCenterOfMassPosition(tmpVec));

        Vector3d vel1 = body1.getVelocityInLocalPoint(relPos1, Stack.newVec());
        Vector3d vel2 = body2.getVelocityInLocalPoint(relPos2, Stack.newVec());
        Vector3d vel = Stack.newVec();
        vel.sub(vel1, vel2);

        double rel_vel = axisNormalOnA.dot(vel);

        // apply displacement correction

        // positional error (zeroth order error)
        tmp.sub(pointInA, pointInB);
        double depth = -(tmp).dot(axisNormalOnA);
        double lo = -1e308;
        double hi = 1e308;

        double minLimit = VectorUtil.getCoord(lowerLimit, limit_index);
        double maxLimit = VectorUtil.getCoord(upperLimit, limit_index);

        // handle the limits
        if (minLimit < maxLimit) {
            if (depth > maxLimit) {
                depth -= maxLimit;
                lo = 0.0;

            } else {
                if (depth < minLimit) {
                    depth -= minLimit;
                    hi = 0.0;
                } else {
                    return 0.0;
                }
            }
        }

        double normalImpulse = limitSoftness * (restitution * depth / timeStep - damping * rel_vel) * jacDiagABInv;

        double oldNormalImpulse = VectorUtil.getCoord(accumulatedImpulse, limit_index);
        double sum = oldNormalImpulse + normalImpulse;
        VectorUtil.setCoord(accumulatedImpulse, limit_index, sum > hi ? 0.0 : sum < lo ? 0.0 : sum);
        normalImpulse = VectorUtil.getCoord(accumulatedImpulse, limit_index) - oldNormalImpulse;

        Vector3d impulseVector = Stack.newVec();
        impulseVector.scale(normalImpulse, axisNormalOnA);
        body1.applyImpulse(impulseVector, relPos1);

        tmp.negate(impulseVector);
        body2.applyImpulse(tmp, relPos2);

        Stack.subVec(8);
        return normalImpulse;
    }

}
