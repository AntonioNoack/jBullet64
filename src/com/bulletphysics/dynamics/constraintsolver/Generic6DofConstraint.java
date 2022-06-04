
/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le�n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/
package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
/*!

 */

/**
 * Generic6DofConstraint between two rigidbodies each with a pivot point that descibes
 * the axis location in local space.<p>
 * <p>
 * Generic6DofConstraint can leave any of the 6 degree of freedom "free" or "locked".
 * Currently this limit supports rotational motors.<br>
 *
 * <ul>
 * <li>For linear limits, use {@link #setLinearUpperLimit}, {@link #setLinearLowerLimit}.
 * You can set the parameters with the {@link TranslationalLimitMotor} structure accsesible
 * through the {@link #getTranslationalLimitMotor} method.
 * At this moment translational motors are not supported. May be in the future.</li>
 *
 * <li>For angular limits, use the {@link RotationalLimitMotor} structure for configuring
 * the limit. This is accessible through {@link #getRotationalLimitMotor} method,
 * this brings support for limit parameters and motors.</li>
 *
 * <li>Angulars limits have these possible ranges:
 * <table border="1">
 * <tr>
 * 	<td><b>AXIS</b></td>
 * 	<td><b>MIN ANGLE</b></td>
 * 	<td><b>MAX ANGLE</b></td>
 * </tr><tr>
 * 	<td>X</td>
 * 		<td>-PI</td>
 * 		<td>PI</td>
 * </tr><tr>
 * 	<td>Y</td>
 * 		<td>-PI/2</td>
 * 		<td>PI/2</td>
 * </tr><tr>
 * 	<td>Z</td>
 * 		<td>-PI/2</td>
 * 		<td>PI/2</td>
 * </tr>
 * </table>
 * </li>
 * </ul>
 *
 * @author jezek2
 */
public class Generic6DofConstraint extends TypedConstraint {

    protected final Transform frameInA = new Transform(); //!< the constraint space w.r.t body A
    protected final Transform frameInB = new Transform(); //!< the constraint space w.r.t body B

    private final JacobianEntry[] jacLinear/*[3]*/ = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; //!< 3 orthogonal linear constraints
    private final JacobianEntry[] jacAng/*[3]*/ = new JacobianEntry[]{new JacobianEntry(), new JacobianEntry(), new JacobianEntry()}; //!< 3 orthogonal angular constraints

    private final TranslationalLimitMotor linearLimits = new TranslationalLimitMotor();

    private final RotationalLimitMotor[] angularLimits/*[3]*/ = new RotationalLimitMotor[]{new RotationalLimitMotor(), new RotationalLimitMotor(), new RotationalLimitMotor()};

    private final Transform calculatedTransformA = new Transform();
    private final Transform calculatedTransformB = new Transform();
    private final Vector3d calculatedAxisAngleDiff = new Vector3d();
    private final Vector3d[] calculatedAxis/*[3]*/ = new Vector3d[]{new Vector3d(), new Vector3d(), new Vector3d()};

    private final Vector3d anchorPos = new Vector3d(); // point between pivots of bodies A and B to solve linear axes

    public boolean useLinearReferenceFrameA;

    public Generic6DofConstraint() {
        super(TypedConstraintType.D6_CONSTRAINT_TYPE);
        useLinearReferenceFrameA = true;
    }

    public Generic6DofConstraint(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB, boolean useLinearReferenceFrameA) {
        super(TypedConstraintType.D6_CONSTRAINT_TYPE, rbA, rbB);
        this.frameInA.set(frameInA);
        this.frameInB.set(frameInB);
        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
    }

    private static double getMatrixElem(Matrix3d mat, int index) {
        int i = index % 3;
        int j = index / 3;
        return mat.getElement(i, j);
    }

    /**
     * MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
     */
    private static boolean matrixToEulerXYZ(Matrix3d mat, Vector3d xyz) {
        //	// rot =  cy*cz          -cy*sz           sy
        //	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
        //	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
        //

        if (getMatrixElem(mat, 2) < 1.0f) {
            if (getMatrixElem(mat, 2) > -1.0f) {
                xyz.x = Math.atan2(-getMatrixElem(mat, 5), getMatrixElem(mat, 8));
                xyz.y = Math.asin(getMatrixElem(mat, 2));
                xyz.z = Math.atan2(-getMatrixElem(mat, 1), getMatrixElem(mat, 0));
                return true;
            } else {
                // WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
                xyz.x = -Math.atan2(getMatrixElem(mat, 3), getMatrixElem(mat, 4));
                xyz.y = -BulletGlobals.SIMD_HALF_PI;
                xyz.z = 0.0f;
                return false;
            }
        } else {
            // WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
            xyz.x = Math.atan2(getMatrixElem(mat, 3), getMatrixElem(mat, 4));
            xyz.y = BulletGlobals.SIMD_HALF_PI;
            xyz.z = 0.0f;
        }

        return false;
    }

    /**
     * Calcs the euler angles between the two bodies.
     */
    protected void calculateAngleInfo() {
        Matrix3d mat = Stack.newMat();

        Matrix3d relative_frame = Stack.newMat();
        mat.set(calculatedTransformA.basis);
        MatrixUtil.invert(mat);
        relative_frame.mul(mat, calculatedTransformB.basis);

        matrixToEulerXYZ(relative_frame, calculatedAxisAngleDiff);

        // in euler angle mode we do not actually constrain the angular velocity
        // along the axes axis[0] and axis[2] (although we do use axis[1]) :
        //
        //    to get			constrain w2-w1 along		...not
        //    ------			---------------------		------
        //    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
        //    d(angle[1])/dt = 0	ax[1]
        //    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
        //
        // constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
        // to prove the result for angle[0], write the expression for angle[0] from
        // GetInfo1 then take the derivative. to prove this for angle[2] it is
        // easier to take the euler rate expression for d(angle[2])/dt with respect
        // to the components of w and set that to 0.

        Vector3d axis0 = Stack.newVec();
        calculatedTransformB.basis.getColumn(0, axis0);

        Vector3d axis2 = Stack.newVec();
        calculatedTransformA.basis.getColumn(2, axis2);

        calculatedAxis[1].cross(axis2, axis0);
        calculatedAxis[0].cross(calculatedAxis[1], axis2);
        calculatedAxis[2].cross(axis0, calculatedAxis[1]);

        //    if(m_debugDrawer)
        //    {
        //
        //    	char buff[300];
        //		sprintf(buff,"\n X: %.2f ; Y: %.2f ; Z: %.2f ",
        //		m_calculatedAxisAngleDiff[0],
        //		m_calculatedAxisAngleDiff[1],
        //		m_calculatedAxisAngleDiff[2]);
        //    	m_debugDrawer->reportErrorWarning(buff);
        //    }
    }

    /**
     * Calcs global transform of the offsets.<p>
     * Calcs the global transform for the joint offset for body A an B, and also calcs the agle differences between the bodies.
     * <p>
     * See also: Generic6DofConstraint.getCalculatedTransformA, Generic6DofConstraint.getCalculatedTransformB, Generic6DofConstraint.calculateAngleInfo
     */
    public void calculateTransforms() {
        rbA.getCenterOfMassTransform(calculatedTransformA);
        calculatedTransformA.mul(frameInA);

        rbB.getCenterOfMassTransform(calculatedTransformB);
        calculatedTransformB.mul(frameInB);

        calculateAngleInfo();
    }

    protected void buildLinearJacobian(/*JacobianEntry jacLinear*/int jacLinear_index, Vector3d normalWorld, Vector3d pivotAInW, Vector3d pivotBInW) {
        Matrix3d mat1 = rbA.getCenterOfMassBasis(Stack.newMat());
        mat1.transpose();

        Matrix3d mat2 = rbB.getCenterOfMassBasis(Stack.newMat());
        mat2.transpose();

        Vector3d tmpVec = Stack.newVec();

        Vector3d tmp1 = Stack.newVec();
        tmp1.sub(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));

        Vector3d tmp2 = Stack.newVec();
        tmp2.sub(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

        jacLinear[jacLinear_index].init(
                mat1,
                mat2,
                tmp1,
                tmp2,
                normalWorld,
                rbA.getInvInertiaDiagLocal(Stack.newVec()),
                rbA.getInvMass(),
                rbB.getInvInertiaDiagLocal(Stack.newVec()),
                rbB.getInvMass());
    }

    protected void buildAngularJacobian(/*JacobianEntry jacAngular*/int jacAngular_index, Vector3d jointAxisW) {
        Matrix3d mat1 = rbA.getCenterOfMassBasis(Stack.newMat());
        mat1.transpose();

        Matrix3d mat2 = rbB.getCenterOfMassBasis(Stack.newMat());
        mat2.transpose();

        jacAng[jacAngular_index].init(jointAxisW,
                mat1,
                mat2,
                rbA.getInvInertiaDiagLocal(Stack.newVec()),
                rbB.getInvInertiaDiagLocal(Stack.newVec()));
    }

    /**
     * Test angular limit.<p>
     * Calculates angular correction and returns true if limit needs to be corrected.
     * Generic6DofConstraint.buildJacobian must be called previously.
     */
    public boolean testAngularLimitMotor(int axis_index) {
        double angle = VectorUtil.getCoord(calculatedAxisAngleDiff, axis_index);

        // test limits
        angularLimits[axis_index].testLimitValue(angle);
        return angularLimits[axis_index].needApplyTorques();
    }

    @Override
    public void buildJacobian() {
        // Clear accumulated impulses for the next simulation step
        linearLimits.accumulatedImpulse.set(0.0, 0.0, 0.0);
        for (int i = 0; i < 3; i++) {
            angularLimits[i].accumulatedImpulse = 0.0;
        }

        // calculates transform
        calculateTransforms();

        // const btVector3& pivotAInW = m_calculatedTransformA.getOrigin();
        // const btVector3& pivotBInW = m_calculatedTransformB.getOrigin();
        calcAnchorPos();
        Vector3d pivotAInW = Stack.newVec(anchorPos);
        Vector3d pivotBInW = Stack.newVec(anchorPos);

        // not used here
        //    btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
        //    btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

        Vector3d normalWorld = Stack.newVec();
        // linear part
        for (int i = 0; i < 3; i++) {
            if (linearLimits.isLimited(i)) {
                if (useLinearReferenceFrameA) {
                    calculatedTransformA.basis.getColumn(i, normalWorld);
                } else {
                    calculatedTransformB.basis.getColumn(i, normalWorld);
                }

                buildLinearJacobian(
                        /*jacLinear[i]*/i, normalWorld,
                        pivotAInW, pivotBInW);

            }
        }

        // angular part
        for (int i = 0; i < 3; i++) {
            // calculates error angle
            if (testAngularLimitMotor(i)) {
                this.getAxis(i, normalWorld);
                // Create angular atom
                buildAngularJacobian(/*jacAng[i]*/i, normalWorld);
            }
        }
    }

    @Override
    public void solveConstraint(double timeStep) {

        //calculateTransforms();

        int i;

        // linear

        Vector3d pointInA = Stack.newVec(calculatedTransformA.origin);
        Vector3d pointInB = Stack.newVec(calculatedTransformB.origin);

        double jacDiagABInv;
        Vector3d linear_axis = Stack.newVec();
        for (i = 0; i < 3; i++) {
            if (linearLimits.isLimited(i)) {
                jacDiagABInv = 1.0 / jacLinear[i].getDiagonal();

                if (useLinearReferenceFrameA) {
                    calculatedTransformA.basis.getColumn(i, linear_axis);
                } else {
                    calculatedTransformB.basis.getColumn(i, linear_axis);
                }

                linearLimits.solveLinearAxis(
                        timeStep,
                        jacDiagABInv,
                        rbA, pointInA,
                        rbB, pointInB,
                        i, linear_axis, anchorPos);

            }
        }

        // angular
        Vector3d angular_axis = Stack.newVec();
        double angularJacDiagABInv;
        for (i = 0; i < 3; i++) {
            if (angularLimits[i].needApplyTorques()) {
                // get axis
                getAxis(i, angular_axis);

                angularJacDiagABInv = 1.0 / jacAng[i].getDiagonal();

                angularLimits[i].solveAngularLimits(timeStep, angular_axis, angularJacDiagABInv, rbA, rbB);
            }
        }
    }


    public void updateRHS(double timeStep) {
    }

    /**
     * Get the rotation axis in global coordinates.
     * Generic6DofConstraint.buildJacobian must be called previously.
     */
    public Vector3d getAxis(int axis_index, Vector3d out) {
        out.set(calculatedAxis[axis_index]);
        return out;
    }

    /**
     * Get the relative Euler angle.
     * Generic6DofConstraint.buildJacobian must be called previously.
     */
    public double getAngle(int axis_index) {
        return VectorUtil.getCoord(calculatedAxisAngleDiff, axis_index);
    }

    /**
     * Gets the global transform of the offset for body A.<p>
     * See also: Generic6DofConstraint.getFrameOffsetA, Generic6DofConstraint.getFrameOffsetB, Generic6DofConstraint.calculateAngleInfo.
     */
    public Transform getCalculatedTransformA(Transform out) {
        out.set(calculatedTransformA);
        return out;
    }

    /**
     * Gets the global transform of the offset for body B.<p>
     * See also: Generic6DofConstraint.getFrameOffsetA, Generic6DofConstraint.getFrameOffsetB, Generic6DofConstraint.calculateAngleInfo.
     */
    public Transform getCalculatedTransformB(Transform out) {
        out.set(calculatedTransformB);
        return out;
    }

    public Transform getFrameOffsetA(Transform out) {
        out.set(frameInA);
        return out;
    }

    public Transform getFrameOffsetB(Transform out) {
        out.set(frameInB);
        return out;
    }

    public void setLinearLowerLimit(Vector3d linearLower) {
        linearLimits.lowerLimit.set(linearLower);
    }

    public void setLinearUpperLimit(Vector3d linearUpper) {
        linearLimits.upperLimit.set(linearUpper);
    }

    public void setAngularLowerLimit(Vector3d angularLower) {
        angularLimits[0].loLimit = angularLower.x;
        angularLimits[1].loLimit = angularLower.y;
        angularLimits[2].loLimit = angularLower.z;
    }

    public void setAngularUpperLimit(Vector3d angularUpper) {
        angularLimits[0].hiLimit = angularUpper.x;
        angularLimits[1].hiLimit = angularUpper.y;
        angularLimits[2].hiLimit = angularUpper.z;
    }

    /**
     * Retrieves the angular limit informacion.
     */
    public RotationalLimitMotor getRotationalLimitMotor(int index) {
        return angularLimits[index];
    }

    /**
     * Retrieves the limit informacion.
     */
    public TranslationalLimitMotor getTranslationalLimitMotor() {
        return linearLimits;
    }

    /**
     * first 3 are linear, next 3 are angular
     */
    public void setLimit(int axis, double lo, double hi) {
        if (axis < 3) {
            VectorUtil.setCoord(linearLimits.lowerLimit, axis, lo);
            VectorUtil.setCoord(linearLimits.upperLimit, axis, hi);
        } else {
            angularLimits[axis - 3].loLimit = lo;
            angularLimits[axis - 3].hiLimit = hi;
        }
    }

    /**
     * Test limit.<p>
     * - free means upper &lt; lower,<br>
     * - locked means upper == lower<br>
     * - limited means upper &gt; lower<br>
     * - limitIndex: first 3 are linear, next 3 are angular
     */
    public boolean isLimited(int limitIndex) {
        if (limitIndex < 3) {
            return linearLimits.isLimited(limitIndex);

        }
        return angularLimits[limitIndex - 3].isLimited();
    }

    // overridable
    public void calcAnchorPos() {
        double imA = rbA.getInvMass();
        double imB = rbB.getInvMass();
        double weight;
        if (imB == 0.0) {
            weight = 1.0;
        } else {
            weight = imA / (imA + imB);
        }
        Vector3d pA = calculatedTransformA.origin;
        Vector3d pB = calculatedTransformB.origin;

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        tmp1.scale(weight, pA);
        tmp2.scale(1f - weight, pB);
        anchorPos.add(tmp1, tmp2);
    }

}
