
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

    protected final Transform frameInA = new Transform();
    protected final Transform frameInB = new Transform();
    // use frameA fo define limits, if true
    public boolean useLinearReferenceFrameA;
    // linear limits
    protected double lowerLinLimit;
    protected double upperLinLimit;
    // angular limits
    protected double lowerAngLimit;
    protected double upperAngLimit;
    // softness, restitution and damping for different cases
    // DirLin - moving inside linear limits
    // LimLin - hitting linear limit
    // DirAng - moving inside angular limits
    // LimAng - hitting angular limit
    // OrthoLin, OrthoAng - against constraint axis
    protected double softnessDirLin;
    protected double restitutionDirLin;
    protected double dampingDirLin;
    protected double softnessDirAng;
    protected double restitutionDirAng;
    protected double dampingDirAng;
    protected double softnessLimLin;
    protected double restitutionLimLin;
    protected double dampingLimLin;
    protected double softnessLimAng;
    protected double restitutionLimAng;
    protected double dampingLimAng;
    protected double softnessOrthoLin;
    protected double restitutionOrthoLin;
    protected double dampingOrthoLin;
    protected double softnessOrthoAng;
    protected double restitutionOrthoAng;
    protected double dampingOrthoAng;

    // for internal use
    protected boolean solveLinLim;
    protected boolean solveAngLim;

    protected JacobianEntry[] jacLin = new JacobianEntry[/*3*/]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};
    protected double[] jacLinDiagABInv = new double[3];

    protected JacobianEntry[] jacAng = new JacobianEntry[/*3*/]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()};

    protected double timeStep;
    protected final Transform calculatedTransformA = new Transform();
    protected final Transform calculatedTransformB = new Transform();

    protected final Vector3d sliderAxis = new Vector3d();
    protected final Vector3d realPivotAInW = new Vector3d();
    protected final Vector3d realPivotBInW = new Vector3d();
    protected final Vector3d projPivotInW = new Vector3d();
    protected final Vector3d delta = new Vector3d();
    protected final Vector3d depth = new Vector3d();
    protected final Vector3d relPosA = new Vector3d();
    protected final Vector3d relPosB = new Vector3d();

    protected double linPos;

    protected double angDepth;
    protected double kAngle;

    protected boolean poweredLinMotor;
    protected double targetLinMotorVelocity;
    protected double maxLinMotorForce;
    protected double accumulatedLinMotorImpulse;

    protected boolean poweredAngMotor;
    protected double targetAngMotorVelocity;
    protected double maxAngMotorForce;
    protected double accumulatedAngMotorImpulse;

    public SliderConstraint() {
        super(TypedConstraintType.SLIDER_CONSTRAINT_TYPE);
        useLinearReferenceFrameA = true;
        initParams();
    }

    public SliderConstraint(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB, boolean useLinearReferenceFrameA) {
        super(TypedConstraintType.SLIDER_CONSTRAINT_TYPE, rbA, rbB);
        this.frameInA.set(frameInA);
        this.frameInB.set(frameInB);
        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        initParams();
    }

    protected void initParams() {
        lowerLinLimit = 1.0;
        upperLinLimit = -1.0;
        lowerAngLimit = 0.0;
        upperAngLimit = 0.0;
        softnessDirLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionDirLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingDirLin = 0.0;
        softnessDirAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionDirAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingDirAng = 0.0;
        softnessOrthoLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionOrthoLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingOrthoLin = SLIDER_CONSTRAINT_DEF_DAMPING;
        softnessOrthoAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionOrthoAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingOrthoAng = SLIDER_CONSTRAINT_DEF_DAMPING;
        softnessLimLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionLimLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingLimLin = SLIDER_CONSTRAINT_DEF_DAMPING;
        softnessLimAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
        restitutionLimAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
        dampingLimAng = SLIDER_CONSTRAINT_DEF_DAMPING;

        poweredLinMotor = false;
        targetLinMotorVelocity = 0.0;
        maxLinMotorForce = 0.0;
        accumulatedLinMotorImpulse = 0.0;

        poweredAngMotor = false;
        targetAngMotorVelocity = 0.0;
        maxAngMotorForce = 0.0;
        accumulatedAngMotorImpulse = 0.0;
    }

    @Override
    public void buildJacobian() {
        if (useLinearReferenceFrameA) {
            buildJacobianInt(rbA, rbB, frameInA, frameInB);
        } else {
            buildJacobianInt(rbB, rbA, frameInB, frameInA);
        }
    }

    @Override
    public void solveConstraint(double timeStep) {
        this.timeStep = timeStep;
        if (useLinearReferenceFrameA) {
            solveConstraintInt(rbA, rbB);
        } else {
            solveConstraintInt(rbB, rbA);
        }
    }

    @SuppressWarnings("unused")
    public Transform getCalculatedTransformA(Transform out) {
        out.set(calculatedTransformA);
        return out;
    }

    @SuppressWarnings("unused")
    public Transform getCalculatedTransformB(Transform out) {
        out.set(calculatedTransformB);
        return out;
    }

    @SuppressWarnings("unused")
    public Transform getFrameOffsetA(Transform out) {
        out.set(frameInA);
        return out;
    }

    @SuppressWarnings("unused")
    public Transform getFrameOffsetB(Transform out) {
        out.set(frameInB);
        return out;
    }

    @SuppressWarnings("unused")
    public double getLowerLinLimit() {
        return lowerLinLimit;
    }

    public void setLowerLinLimit(double lowerLimit) {
        this.lowerLinLimit = lowerLimit;
    }

    @SuppressWarnings("unused")
    public double getUpperLinLimit() {
        return upperLinLimit;
    }

    public void setUpperLinLimit(double upperLimit) {
        this.upperLinLimit = upperLimit;
    }

    @SuppressWarnings("unused")
    public double getLowerAngLimit() {
        return lowerAngLimit;
    }

    public void setLowerAngLimit(double lowerLimit) {
        this.lowerAngLimit = lowerLimit;
    }

    @SuppressWarnings("unused")
    public double getUpperAngLimit() {
        return upperAngLimit;
    }

    public void setUpperAngLimit(double upperLimit) {
        this.upperAngLimit = upperLimit;
    }

    @SuppressWarnings("unused")
    public boolean getUseLinearReferenceFrameA() {
        return useLinearReferenceFrameA;
    }

    @SuppressWarnings("unused")
    public double getSoftnessDirLin() {
        return softnessDirLin;
    }

    @SuppressWarnings("unused")
    public double getRestitutionDirLin() {
        return restitutionDirLin;
    }

    @SuppressWarnings("unused")
    public double getDampingDirLin() {
        return dampingDirLin;
    }

    @SuppressWarnings("unused")
    public double getSoftnessDirAng() {
        return softnessDirAng;
    }

    @SuppressWarnings("unused")
    public double getRestitutionDirAng() {
        return restitutionDirAng;
    }

    @SuppressWarnings("unused")
    public double getDampingDirAng() {
        return dampingDirAng;
    }

    @SuppressWarnings("unused")
    public double getSoftnessLimLin() {
        return softnessLimLin;
    }

    @SuppressWarnings("unused")
    public double getRestitutionLimLin() {
        return restitutionLimLin;
    }

    @SuppressWarnings("unused")
    public double getDampingLimLin() {
        return dampingLimLin;
    }

    @SuppressWarnings("unused")
    public double getSoftnessLimAng() {
        return softnessLimAng;
    }

    @SuppressWarnings("unused")
    public double getRestitutionLimAng() {
        return restitutionLimAng;
    }

    @SuppressWarnings("unused")
    public double getDampingLimAng() {
        return dampingLimAng;
    }

    @SuppressWarnings("unused")
    public double getSoftnessOrthoLin() {
        return softnessOrthoLin;
    }

    @SuppressWarnings("unused")
    public double getRestitutionOrthoLin() {
        return restitutionOrthoLin;
    }

    @SuppressWarnings("unused")
    public double getDampingOrthoLin() {
        return dampingOrthoLin;
    }

    @SuppressWarnings("unused")
    public double getSoftnessOrthoAng() {
        return softnessOrthoAng;
    }

    @SuppressWarnings("unused")
    public double getRestitutionOrthoAng() {
        return restitutionOrthoAng;
    }

    @SuppressWarnings("unused")
    public double getDampingOrthoAng() {
        return dampingOrthoAng;
    }

    @SuppressWarnings("unused")
    public void setSoftnessDirLin(double softnessDirLin) {
        this.softnessDirLin = softnessDirLin;
    }

    @SuppressWarnings("unused")
    public void setRestitutionDirLin(double restitutionDirLin) {
        this.restitutionDirLin = restitutionDirLin;
    }

    @SuppressWarnings("unused")
    public void setDampingDirLin(double dampingDirLin) {
        this.dampingDirLin = dampingDirLin;
    }

    @SuppressWarnings("unused")
    public void setSoftnessDirAng(double softnessDirAng) {
        this.softnessDirAng = softnessDirAng;
    }

    @SuppressWarnings("unused")
    public void setRestitutionDirAng(double restitutionDirAng) {
        this.restitutionDirAng = restitutionDirAng;
    }

    @SuppressWarnings("unused")
    public void setDampingDirAng(double dampingDirAng) {
        this.dampingDirAng = dampingDirAng;
    }

    @SuppressWarnings("unused")
    public void setSoftnessLimLin(double softnessLimLin) {
        this.softnessLimLin = softnessLimLin;
    }

    @SuppressWarnings("unused")
    public void setRestitutionLimLin(double restitutionLimLin) {
        this.restitutionLimLin = restitutionLimLin;
    }

    @SuppressWarnings("unused")
    public void setDampingLimLin(double dampingLimLin) {
        this.dampingLimLin = dampingLimLin;
    }

    @SuppressWarnings("unused")
    public void setSoftnessLimAng(double softnessLimAng) {
        this.softnessLimAng = softnessLimAng;
    }

    @SuppressWarnings("unused")
    public void setRestitutionLimAng(double restitutionLimAng) {
        this.restitutionLimAng = restitutionLimAng;
    }

    @SuppressWarnings("unused")
    public void setDampingLimAng(double dampingLimAng) {
        this.dampingLimAng = dampingLimAng;
    }

    @SuppressWarnings("unused")
    public void setSoftnessOrthoLin(double softnessOrthoLin) {
        this.softnessOrthoLin = softnessOrthoLin;
    }

    @SuppressWarnings("unused")
    public void setRestitutionOrthoLin(double restitutionOrthoLin) {
        this.restitutionOrthoLin = restitutionOrthoLin;
    }

    @SuppressWarnings("unused")
    public void setDampingOrthoLin(double dampingOrthoLin) {
        this.dampingOrthoLin = dampingOrthoLin;
    }

    @SuppressWarnings("unused")
    public void setSoftnessOrthoAng(double softnessOrthoAng) {
        this.softnessOrthoAng = softnessOrthoAng;
    }

    @SuppressWarnings("unused")
    public void setRestitutionOrthoAng(double restitutionOrthoAng) {
        this.restitutionOrthoAng = restitutionOrthoAng;
    }

    @SuppressWarnings("unused")
    public void setDampingOrthoAng(double dampingOrthoAng) {
        this.dampingOrthoAng = dampingOrthoAng;
    }

    @SuppressWarnings("unused")
    public void setPoweredLinMotor(boolean onOff) {
        this.poweredLinMotor = onOff;
    }

    @SuppressWarnings("unused")
    public boolean getPoweredLinMotor() {
        return poweredLinMotor;
    }

    @SuppressWarnings("unused")
    public void setTargetLinMotorVelocity(double targetLinMotorVelocity) {
        this.targetLinMotorVelocity = targetLinMotorVelocity;
    }

    @SuppressWarnings("unused")
    public double getTargetLinMotorVelocity() {
        return targetLinMotorVelocity;
    }

    @SuppressWarnings("unused")
    public void setMaxLinMotorForce(double maxLinMotorForce) {
        this.maxLinMotorForce = maxLinMotorForce;
    }

    @SuppressWarnings("unused")
    public double getMaxLinMotorForce() {
        return maxLinMotorForce;
    }

    @SuppressWarnings("unused")
    public void setPoweredAngMotor(boolean onOff) {
        this.poweredAngMotor = onOff;
    }

    @SuppressWarnings("unused")
    public boolean getPoweredAngMotor() {
        return poweredAngMotor;
    }

    @SuppressWarnings("unused")
    public void setTargetAngMotorVelocity(double targetAngMotorVelocity) {
        this.targetAngMotorVelocity = targetAngMotorVelocity;
    }

    @SuppressWarnings("unused")
    public double getTargetAngMotorVelocity() {
        return targetAngMotorVelocity;
    }

    @SuppressWarnings("unused")
    public void setMaxAngMotorForce(double maxAngMotorForce) {
        this.maxAngMotorForce = maxAngMotorForce;
    }

    @SuppressWarnings("unused")
    public double getMaxAngMotorForce() {
        return this.maxAngMotorForce;
    }

    @SuppressWarnings("unused")
    public double getLinearPos() {
        return this.linPos;
    }

    // access for ODE solver

    @SuppressWarnings("unused")
    public boolean getSolveLinLimit() {
        return solveLinLim;
    }

    @SuppressWarnings("unused")
    public double getLinDepth() {
        return depth.x;
    }

    @SuppressWarnings("unused")
    public boolean getSolveAngLimit() {
        return solveAngLim;
    }

    @SuppressWarnings("unused")
    public double getAngDepth() {
        return angDepth;
    }

    // internal

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
                    rbA.getInvMass(),
                    rbB.getInvInertiaDiagLocal(tmp2),
                    rbB.getInvMass());
            jacLinDiagABInv[i] = 1.0 / jacLin[i].getDiagonal();
            VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
        }
        testLinLimits();

        // angular part
        for (int i = 0; i < 3; i++) {
            calculatedTransformA.basis.getColumn(i, normalWorld);

            Matrix3d mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
            mat1.transpose();

            Matrix3d mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
            mat2.transpose();

            jacAng[i].init(
                    normalWorld,
                    mat1,
                    mat2,
                    rbA.getInvInertiaDiagLocal(tmp),
                    rbB.getInvInertiaDiagLocal(tmp2));
        }
        testAngLimits();

        Vector3d axisA = Stack.newVec();
        calculatedTransformA.basis.getColumn(0, axisA);
        kAngle = 1.0 / (rbA.computeAngularImpulseDenominator(axisA) + rbB.computeAngularImpulseDenominator(axisA));
        // clear accumulator for motors
        accumulatedLinMotorImpulse = 0.0;
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
            double softness = (i != 0) ? softnessOrthoLin : (solveLinLim ? softnessLimLin : softnessDirLin);
            double restitution = (i != 0) ? restitutionOrthoLin : (solveLinLim ? restitutionLimLin : restitutionDirLin);
            double damping = (i != 0) ? dampingOrthoLin : (solveLinLim ? dampingLimLin : dampingDirLin);
            // calculate and apply impulse
            double normalImpulse = softness * (restitution * depth / timeStep - damping * relVel) * jacLinDiagABInv[i];
            if (Math.abs(normalImpulse) > getBreakingImpulseThreshold()) {
                setBroken(true);
                break;
            }

            impulseVector.scale(normalImpulse, normal);
            rbA.applyImpulse(impulseVector, relPosA);
            tmp.negate(impulseVector);
            rbB.applyImpulse(tmp, relPosB);

            if (poweredLinMotor && (i == 0)) {
                // apply linear motor
                if (accumulatedLinMotorImpulse < maxLinMotorForce) {
                    double desiredMotorVel = targetLinMotorVelocity;
                    double motorRelVel = desiredMotorVel + relVel;
                    normalImpulse = -motorRelVel * jacLinDiagABInv[i];
                    // clamp accumulated impulse
                    double newAcc = accumulatedLinMotorImpulse + Math.abs(normalImpulse);
                    if (newAcc > maxLinMotorForce) {
                        newAcc = maxLinMotorForce;
                    }
                    double del = newAcc - accumulatedLinMotorImpulse;
                    if (normalImpulse < 0.0) {
                        normalImpulse = -del;
                    } else {
                        normalImpulse = del;
                    }
                    accumulatedLinMotorImpulse = newAcc;
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
            velRelOrthogonal.scale((1.0 / denominator) * dampingOrthoAng * softnessOrthoAng);
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
            angularError.scale((1.0 / denominator) * restitutionOrthoAng * softnessOrthoAng);
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
            impulseMag = tmp.dot(axisA) * dampingLimAng + angDepth * restitutionLimAng / timeStep;
            impulseMag *= kAngle * softnessLimAng;
        } else {
            tmp.sub(angVelB, angVelA);
            impulseMag = tmp.dot(axisA) * dampingDirAng + angDepth * restitutionDirAng / timeStep;
            impulseMag *= kAngle * softnessDirAng;
        }

        if (Math.abs(impulseMag) > getBreakingImpulseThreshold()) {
            setBroken(true);
            impulseMag = 0.0;
        }

        Vector3d impulse = Stack.newVec();
        impulse.scale(impulseMag, axisA);
        rbA.applyTorqueImpulse(impulse);
        tmp.negate(impulse);
        rbB.applyTorqueImpulse(tmp);

        // apply angular motor
        if (poweredAngMotor) {
            if (accumulatedAngMotorImpulse < maxAngMotorForce) {
                Vector3d velRel = Stack.newVec();
                velRel.sub(angVelAroundAxisA, angVelAroundAxisB);
                double projRelVel = velRel.dot(axisA);

                double desiredMotorVel = targetAngMotorVelocity;
                double motorRelVel = desiredMotorVel - projRelVel;

                double angImpulse = kAngle * motorRelVel;
                if (Math.abs(angImpulse) > getBreakingImpulseThreshold()) {
                    setBroken(true);
                    angImpulse = 0.0;
                }

                // clamp accumulated impulse
                double newAcc = accumulatedAngMotorImpulse + Math.abs(angImpulse);
                if (newAcc > maxAngMotorForce) {
                    newAcc = maxAngMotorForce;
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
        linPos = depth.x;
        if (lowerLinLimit <= upperLinLimit) {
            if (depth.x > upperLinLimit) {
                depth.x -= upperLinLimit;
                solveLinLim = true;
            } else if (depth.x < lowerLinLimit) {
                depth.x -= lowerLinLimit;
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
        if (lowerAngLimit <= upperAngLimit) {
            Vector3d axisA0 = Stack.newVec();
            calculatedTransformA.basis.getColumn(1, axisA0);
            Vector3d axisA1 = Stack.newVec();
            calculatedTransformA.basis.getColumn(2, axisA1);
            Vector3d axisB0 = Stack.newVec();
            calculatedTransformB.basis.getColumn(1, axisB0);

            double rot = Math.atan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
            if (rot < lowerAngLimit) {
                angDepth = rot - lowerAngLimit;
                solveAngLim = true;
            } else if (rot > upperAngLimit) {
                angDepth = rot - upperAngLimit;
                solveAngLim = true;
            }
        }
    }
}
