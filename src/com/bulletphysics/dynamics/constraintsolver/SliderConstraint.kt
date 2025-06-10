
/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
*/
package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

// JAVA NOTE: SliderConstraint from 2.71

/**
 * @author jezek2
 */
public class SliderConstraint extends TypedConstraint {

    public static final double SLIDER_CONSTRAINT_DEF_SOFTNESS = 1.0;
    public static final double SLIDER_CONSTRAINT_DEF_DAMPING = 1.0;
    public static final double SLIDER_CONSTRAINT_DEF_RESTITUTION = 0.7;

    public final Transform frameOffsetA = new Transform();
    public final Transform frameOffsetB = new Transform();
    // use frameA fo define limits, if true
    public boolean useLinearReferenceFrameA;
    // linear limits
    public double lowerLinearLimit;
    public double upperLinearLimit;
    // angular limits
    public double lowerAngularLimit;
    public double upperAngularLimit;
    // softness, restitution and damping for different cases
    // DirLin - moving inside linear limits
    // LimLin - hitting linear limit
    // DirAng - moving inside angular limits
    // LimAng - hitting angular limit
    // OrthoLin, OrthoAng - against constraint axis
    public double softnessDirLinear;
    public double restitutionDirLinear;
    public double dampingDirLinear;
    public double softnessDirAngular;
    public double restitutionDirAngular;
    public double dampingDirAngular;
    public double softnessLimitLinear;
    public double restitutionLimitLinear;
    public double dampingLimitLinear;
    public double softnessLimitAngular;
    public double restitutionLimitAngular;
    public double dampingLimitAngular;
    public double softnessOrthogonalLinear;
    public double restitutionOrthogonalLinear;
    public double dampingOrthogonalLinear;
    public double softnessOrthogonalAngular;
    public double restitutionOrthogonalAngular;
    public double dampingOrthogonalAngular;

    // for internal use
    private boolean solveLinLim;
    private boolean solveAngLim;

    private final JacobianEntry[] jacLin = new JacobianEntry[/*3*/]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};
    private final double[] jacLinDiagABInv = new double[3];

    private double timeStep;
    public final Transform calculatedTransformA = new Transform();
    public final Transform calculatedTransformB = new Transform();

    private final Vector3d sliderAxis = new Vector3d();
    private final Vector3d realPivotAInW = new Vector3d();
    private final Vector3d realPivotBInW = new Vector3d();
    private final Vector3d projPivotInW = new Vector3d();
    private final Vector3d delta = new Vector3d();
    private final Vector3d depth = new Vector3d();
    private final Vector3d relPosA = new Vector3d();
    private final Vector3d relPosB = new Vector3d();

    public double linearPosition;

    private double angDepth;
    private double kAngle;

    public boolean poweredLinearMotor;
    public double targetLinearMotorVelocity;
    public double maxLinearMotorForce;
    private double accumulatedLinearMotorImpulse;

    public boolean poweredAngularMotor;
    public double targetAngularMotorVelocity;
    public double maxAngularMotorForce;
    private double accumulatedAngMotorImpulse;

    @SuppressWarnings("unused")
    public SliderConstraint() {
        useLinearReferenceFrameA = true;
        initParams();
    }

    public SliderConstraint(RigidBody rbA, RigidBody rbB, Transform frameOffsetA, Transform frameOffsetB, boolean useLinearReferenceFrameA) {
        super(rbA, rbB);
        this.frameOffsetA.set(frameOffsetA);
        this.frameOffsetB.set(frameOffsetB);
        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        initParams();
    }

    protected void initParams() {
        lowerLinearLimit = 1.0;
        upperLinearLimit = -1.0;
        lowerAngularLimit = 0.0;
        upperAngularLimit = 0.0;
        softnessDirLinear = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionDirLinear = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingDirLinear = 0.0;
        softnessDirAngular = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionDirAngular = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingDirAngular = 0.0;
        softnessOrthogonalLinear = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionOrthogonalLinear = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingOrthogonalLinear = SLIDER_CONSTRAINT_DEF_DAMPING;
        softnessOrthogonalAngular = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionOrthogonalAngular = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingOrthogonalAngular = SLIDER_CONSTRAINT_DEF_DAMPING;
        softnessLimitLinear = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionLimitLinear = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingLimitLinear = SLIDER_CONSTRAINT_DEF_DAMPING;
        softnessLimitAngular = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionLimitAngular = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingLimitAngular = SLIDER_CONSTRAINT_DEF_DAMPING;

        poweredLinearMotor = false;
        targetLinearMotorVelocity = 0.0;
        maxLinearMotorForce = 0.0;
        accumulatedLinearMotorImpulse = 0.0;

        poweredAngularMotor = false;
        targetAngularMotorVelocity = 0.0;
        maxAngularMotorForce = 0.0;
        accumulatedAngMotorImpulse = 0.0;
    }

    @Override
    public void buildJacobian() {
        if (useLinearReferenceFrameA) {
            buildJacobianInt(rigidBodyA, rigidBodyB, frameOffsetA, frameOffsetB);
        } else {
            buildJacobianInt(rigidBodyB, rigidBodyA, frameOffsetB, frameOffsetA);
        }
    }

    @Override
    public void solveConstraint(double timeStep) {
        this.timeStep = timeStep;
        if (useLinearReferenceFrameA) {
            solveConstraintInt(rigidBodyA, rigidBodyB);
        } else {
            solveConstraintInt(rigidBodyB, rigidBodyA);
        }
    }

    public void buildJacobianInt(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB) {
        Transform tmpTrans = Stack.newTrans();
        Transform tmpTrans1 = Stack.newTrans();
        Transform tmpTrans2 = Stack.newTrans();
        Vector3d tmp = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        // calculate transforms
        calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
        calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
        realPivotAInW.set(calculatedTransformA.origin);
        realPivotBInW.set(calculatedTransformB.origin);
        calculatedTransformA.basis.getColumn(0, tmp);
        sliderAxis.set(tmp); // along X
        delta.sub(realPivotBInW, realPivotAInW);
        projPivotInW.scaleAdd(sliderAxis.dot(delta), sliderAxis, realPivotAInW);
        relPosA.sub(projPivotInW, rbA.getCenterOfMassPosition(tmp));
        relPosB.sub(realPivotBInW, rbB.getCenterOfMassPosition(tmp));
        Vector3d normalWorld = Stack.newVec();

        // linear part
        for (int i = 0; i < 3; i++) {
            calculatedTransformA.basis.getColumn(i, normalWorld);

            Matrix3d mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
            mat1.transpose();

            Matrix3d mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
            mat2.transpose();

            jacLin[i].init(
                    mat1,
                    mat2,
                    relPosA,
                    relPosB,
                    normalWorld,
                    rbA.getInvInertiaDiagLocal(tmp),
                    rbA.inverseMass,
                    rbB.getInvInertiaDiagLocal(tmp2),
                    rbB.inverseMass);
            jacLinDiagABInv[i] = 1.0 / jacLin[i].getDiagonal();
            VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
        }
        testLinLimits();

        // angular part
        testAngLimits();

        Vector3d axisA = Stack.newVec();
        calculatedTransformA.basis.getColumn(0, axisA);
        kAngle = 1.0 / (rbA.computeAngularImpulseDenominator(axisA) + rbB.computeAngularImpulseDenominator(axisA));
        // clear accumulator for motors
        accumulatedLinearMotorImpulse = 0.0;
        accumulatedAngMotorImpulse = 0.0;
    }

    public void solveConstraintInt(RigidBody rbA, RigidBody rbB) {
        Vector3d tmp = Stack.newVec();

        // linear
        Vector3d velA = rbA.getVelocityInLocalPoint(relPosA, Stack.newVec());
        Vector3d velB = rbB.getVelocityInLocalPoint(relPosB, Stack.newVec());
        Vector3d vel = Stack.newVec();
        vel.sub(velA, velB);

        Vector3d impulseVector = Stack.newVec();

        for (int i = 0; i < 3; i++) {
            Vector3d normal = jacLin[i].linearJointAxis;
            double relVel = normal.dot(vel);
            // calculate positional error
            double depth = VectorUtil.getCoord(this.depth, i);
            // get parameters
            double softness = (i != 0) ? softnessOrthogonalLinear : (solveLinLim ? softnessLimitLinear : softnessDirLinear);
            double restitution = (i != 0) ? restitutionOrthogonalLinear : (solveLinLim ? restitutionLimitLinear : restitutionDirLinear);
            double damping = (i != 0) ? dampingOrthogonalLinear : (solveLinLim ? dampingLimitLinear : dampingDirLinear);
            // calculate and apply impulse
            double normalImpulse = softness * (restitution * depth / timeStep - damping * relVel) * jacLinDiagABInv[i];
            if (Math.abs(normalImpulse) > breakingImpulseThreshold) {
                setBroken(true);
                break;
            }

            impulseVector.scale(normalImpulse, normal);
            rbA.applyImpulse(impulseVector, relPosA);
            tmp.negate(impulseVector);
            rbB.applyImpulse(tmp, relPosB);

            if (poweredLinearMotor && (i == 0)) {
                // apply linear motor
                if (accumulatedLinearMotorImpulse < maxLinearMotorForce) {
                    double desiredMotorVel = targetLinearMotorVelocity;
                    double motorRelVel = desiredMotorVel + relVel;
                    normalImpulse = -motorRelVel * jacLinDiagABInv[i];
                    // clamp accumulated impulse
                    double newAcc = accumulatedLinearMotorImpulse + Math.abs(normalImpulse);
                    if (newAcc > maxLinearMotorForce) {
                        newAcc = maxLinearMotorForce;
                    }
                    double del = newAcc - accumulatedLinearMotorImpulse;
                    if (normalImpulse < 0.0) {
                        normalImpulse = -del;
                    } else {
                        normalImpulse = del;
                    }
                    accumulatedLinearMotorImpulse = newAcc;
                    // apply clamped impulse
                    impulseVector.scale(normalImpulse, normal);
                    rbA.applyImpulse(impulseVector, relPosA);
                    tmp.negate(impulseVector);
                    rbB.applyImpulse(tmp, relPosB);
                }
            }
        }

        // angular
        // get axes in world space
        Vector3d axisA = Stack.newVec();
        calculatedTransformA.basis.getColumn(0, axisA);
        Vector3d axisB = Stack.newVec();
        calculatedTransformB.basis.getColumn(0, axisB);

        Vector3d angVelA = rbA.getAngularVelocity(Stack.newVec());
        Vector3d angVelB = rbB.getAngularVelocity(Stack.newVec());

        Vector3d angVelAroundAxisA = Stack.newVec();
        angVelAroundAxisA.scale(axisA.dot(angVelA), axisA);
        Vector3d angVelAroundAxisB = Stack.newVec();
        angVelAroundAxisB.scale(axisB.dot(angVelB), axisB);

        Vector3d angleAOrthogonal = Stack.newVec();
        angleAOrthogonal.sub(angVelA, angVelAroundAxisA);
        Vector3d angleBOrthogonal = Stack.newVec();
        angleBOrthogonal.sub(angVelB, angVelAroundAxisB);
        Vector3d velRelOrthogonal = Stack.newVec();
        velRelOrthogonal.sub(angleAOrthogonal, angleBOrthogonal);

        // solve orthogonal angular velocity correction
        double len = velRelOrthogonal.length();
        if (len > 0.00001) {
            Vector3d normal = Stack.newVec();
            normal.normalize(velRelOrthogonal);
            double denominator = rbA.computeAngularImpulseDenominator(normal) + rbB.computeAngularImpulseDenominator(normal);
            velRelOrthogonal.scale((1.0 / denominator) * dampingOrthogonalAngular * softnessOrthogonalAngular);
        }

        // solve angular positional correction
        Vector3d angularError = Stack.newVec();
        angularError.cross(axisA, axisB);
        angularError.scale(1.0 / timeStep);
        double len2 = angularError.length();
        if (len2 > 0.00001) {
            Vector3d normal2 = Stack.newVec();
            normal2.normalize(angularError);
            double denominator = rbA.computeAngularImpulseDenominator(normal2) + rbB.computeAngularImpulseDenominator(normal2);
            angularError.scale((1.0 / denominator) * restitutionOrthogonalAngular * softnessOrthogonalAngular);
        }

        // apply impulse
        tmp.negate(velRelOrthogonal);
        tmp.add(angularError);
        rbA.applyTorqueImpulse(tmp);
        tmp.sub(velRelOrthogonal, angularError);
        rbB.applyTorqueImpulse(tmp);
        double impulseMag;

        // solve angular limits
        if (solveAngLim) {
            tmp.sub(angVelB, angVelA);
            impulseMag = tmp.dot(axisA) * dampingLimitAngular + angDepth * restitutionLimitAngular / timeStep;
            impulseMag *= kAngle * softnessLimitAngular;
        } else {
            tmp.sub(angVelB, angVelA);
            impulseMag = tmp.dot(axisA) * dampingDirAngular + angDepth * restitutionDirAngular / timeStep;
            impulseMag *= kAngle * softnessDirAngular;
        }

        if (Math.abs(impulseMag) > breakingImpulseThreshold) {
            setBroken(true);
            impulseMag = 0.0;
        }

        Vector3d impulse = Stack.newVec();
        impulse.scale(impulseMag, axisA);
        rbA.applyTorqueImpulse(impulse);
        tmp.negate(impulse);
        rbB.applyTorqueImpulse(tmp);

        // apply angular motor
        if (poweredAngularMotor) {
            if (accumulatedAngMotorImpulse < maxAngularMotorForce) {
                Vector3d velRel = Stack.newVec();
                velRel.sub(angVelAroundAxisA, angVelAroundAxisB);
                double projRelVel = velRel.dot(axisA);

                double desiredMotorVel = targetAngularMotorVelocity;
                double motorRelVel = desiredMotorVel - projRelVel;

                double angImpulse = kAngle * motorRelVel;
                if (Math.abs(angImpulse) > breakingImpulseThreshold) {
                    setBroken(true);
                    angImpulse = 0.0;
                }

                // clamp accumulated impulse
                double newAcc = accumulatedAngMotorImpulse + Math.abs(angImpulse);
                if (newAcc > maxAngularMotorForce) {
                    newAcc = maxAngularMotorForce;
                }
                double del = newAcc - accumulatedAngMotorImpulse;
                if (angImpulse < 0.0) {
                    angImpulse = -del;
                } else {
                    angImpulse = del;
                }
                accumulatedAngMotorImpulse = newAcc;

                // apply clamped impulse
                Vector3d motorImp = Stack.newVec();
                motorImp.scale(angImpulse, axisA);
                rbA.applyTorqueImpulse(motorImp);
                tmp.negate(motorImp);
                rbB.applyTorqueImpulse(tmp);
            }
        }
    }

    public void testLinLimits() {
        solveLinLim = false;
        linearPosition = depth.x;
        if (lowerLinearLimit <= upperLinearLimit) {
            if (depth.x > upperLinearLimit) {
                depth.x -= upperLinearLimit;
                solveLinLim = true;
            } else if (depth.x < lowerLinearLimit) {
                depth.x -= lowerLinearLimit;
                solveLinLim = true;
            } else {
                depth.x = 0.0;
            }
        } else {
            depth.x = 0.0;
        }
    }

    public void testAngLimits() {
        angDepth = 0.0;
        solveAngLim = false;
        if (lowerAngularLimit <= upperAngularLimit) {
            Vector3d axisA0 = Stack.newVec();
            calculatedTransformA.basis.getColumn(1, axisA0);
            Vector3d axisA1 = Stack.newVec();
            calculatedTransformA.basis.getColumn(2, axisA1);
            Vector3d axisB0 = Stack.newVec();
            calculatedTransformB.basis.getColumn(1, axisB0);

            double rot = Math.atan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
            if (rot < lowerAngularLimit) {
                angDepth = rot - lowerAngularLimit;
                solveAngLim = true;
            } else if (rot > upperAngularLimit) {
                angDepth = rot - upperAngularLimit;
                solveAngLim = true;
            }
            Stack.subVec(3);
        }
    }
}
