/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.dynamics;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.linearmath.*;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;
import cz.advel.stack.StaticAlloc;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

/**
 * RigidBody is the main class for rigid body objects. It is derived from
 * {@link CollisionObject}, so it keeps reference to {@link CollisionShape}.<p>
 * <p>
 * It is recommended for performance and memory use to share {@link CollisionShape}
 * objects whenever possible.<p>
 * <p>
 * There are 3 types of rigid bodies:<br>
 * <ol>
 * <li>Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.</li>
 * <li>Fixed objects with zero mass. They are not moving (basically collision objects).</li>
 * <li>Kinematic objects, which are objects without mass, but the user can move them. There
 *     is on-way interaction, and Bullet calculates a velocity based on the timestep and
 *     previous and current world transform.</li>
 * </ol>
 * <p>
 * Bullet automatically deactivates dynamic rigid bodies, when the velocity is below
 * a threshold for a given time.<p>
 * <p>
 * Deactivated (sleeping) rigid bodies don't take any processing time, except a minor
 * broadphase collision detection impact (to allow active objects to activate/wake up
 * sleeping objects).
 *
 * @author jezek2
 */
public class RigidBody extends CollisionObject {

    private static final double MAX_ANGVEL = BulletGlobals.SIMD_HALF_PI;

    private final Matrix3d invInertiaTensorWorld = new Matrix3d();
    private final Vector3d linearVelocity = new Vector3d();
    private final Vector3d angularVelocity = new Vector3d();
    private double inverseMass;
    private double angularFactor;

    private final Vector3d gravity = new Vector3d();
    private final Vector3d invInertiaLocal = new Vector3d();
    private final Vector3d totalForce = new Vector3d();
    private final Vector3d totalTorque = new Vector3d();

    private double linearDamping;
    private double angularDamping;

    private boolean additionalDamping;
    private double additionalDampingFactor;
    private double additionalLinearDampingThresholdSqr;
    private double additionalAngularDampingThresholdSqr;
    private double additionalAngularDampingFactor;

    private double linearSleepingThreshold;
    private double angularSleepingThreshold;

    // optionalMotionState allows to automatic synchronize the world transform for active objects
    private MotionState optionalMotionState;

    // keep track of typed constraints referencing this rigid body
    private final ObjectArrayList<TypedConstraint> constraintRefs = new ObjectArrayList<TypedConstraint>();

    // for experimental overriding of friction/contact solver func
    public int contactSolverType;
    public int frictionSolverType;

    private static int uniqueId = 0;
    public int debugBodyId;

    public RigidBody(RigidBodyConstructionInfo constructionInfo) {
        setupRigidBody(constructionInfo);
    }

    public RigidBody(double mass, MotionState motionState, CollisionShape collisionShape) {
        this(mass, motionState, collisionShape, new Vector3d(0f, 0f, 0f));
    }

    public RigidBody(double mass, MotionState motionState, CollisionShape collisionShape, Vector3d localInertia) {
        RigidBodyConstructionInfo cinfo = new RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia);
        setupRigidBody(cinfo);
    }

    private void setupRigidBody(RigidBodyConstructionInfo constructionInfo) {
        internalType = CollisionObjectType.RIGID_BODY;

        linearVelocity.set(0f, 0f, 0f);
        angularVelocity.set(0f, 0f, 0f);
        angularFactor = 1f;
        gravity.set(0f, 0f, 0f);
        totalForce.set(0f, 0f, 0f);
        totalTorque.set(0f, 0f, 0f);
        linearDamping = 0f;
        angularDamping = 0.5f;
        linearSleepingThreshold = constructionInfo.linearSleepingThreshold;
        angularSleepingThreshold = constructionInfo.angularSleepingThreshold;
        optionalMotionState = constructionInfo.motionState;
        contactSolverType = 0;
        frictionSolverType = 0;
        additionalDamping = constructionInfo.additionalDamping;
        additionalDampingFactor = constructionInfo.additionalDampingFactor;
        additionalLinearDampingThresholdSqr = constructionInfo.additionalLinearDampingThresholdSqr;
        additionalAngularDampingThresholdSqr = constructionInfo.additionalAngularDampingThresholdSqr;
        additionalAngularDampingFactor = constructionInfo.additionalAngularDampingFactor;

        if (optionalMotionState != null) {
            optionalMotionState.getWorldTransform(worldTransform);
        } else {
            worldTransform.set(constructionInfo.startWorldTransform);
        }

        interpolationWorldTransform.set(worldTransform);
        interpolationLinearVelocity.set(0f, 0f, 0f);
        interpolationAngularVelocity.set(0f, 0f, 0f);

        // moved to CollisionObject
        friction = constructionInfo.friction;
        restitution = constructionInfo.restitution;

        setCollisionShape(constructionInfo.collisionShape);
        debugBodyId = uniqueId++;

        setMassProps(constructionInfo.mass, constructionInfo.localInertia);
        setDamping(constructionInfo.linearDamping, constructionInfo.angularDamping);
        updateInertiaTensor();
    }

    public void destroy() {
        // No constraints should point to this rigidbody
        // Remove constraints from the dynamics world before you delete the related rigidbodies.
        assert (constraintRefs.size() == 0);
    }

    public void proceedToTransform(Transform newTrans) {
        setCenterOfMassTransform(newTrans);
    }

    /**
     * To keep collision detection and dynamics separate we don't store a rigidbody pointer,
     * but a rigidbody is derived from CollisionObject, so we can safely perform an upcast.
     */
    public static RigidBody upcast(CollisionObject colObj) {
        if (colObj.getInternalType() == CollisionObjectType.RIGID_BODY) {
            return (RigidBody) colObj;
        }
        return null;
    }

    /**
     * Continuous collision detection needs prediction.
     */
    public void predictIntegratedTransform(double timeStep, Transform predictedTransform) {
        TransformUtil.integrateTransform(worldTransform, linearVelocity, angularVelocity, timeStep, predictedTransform);
    }

    public void saveKinematicState(double timeStep) {
        //todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
        if (timeStep != 0f) {
            //if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
            if (getMotionState() != null) {
                getMotionState().getWorldTransform(worldTransform);
            }
            //Vector3d linVel = new Vector3d(), angVel = new Vector3d();

            TransformUtil.calculateVelocity(interpolationWorldTransform, worldTransform, timeStep, linearVelocity, angularVelocity);
            interpolationLinearVelocity.set(linearVelocity);
            interpolationAngularVelocity.set(angularVelocity);
            interpolationWorldTransform.set(worldTransform);
            //printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
        }
    }

    public void applyGravity() {
        if (isStaticOrKinematicObject())
            return;

        applyCentralForce(gravity);
    }

    public void setGravity(Vector3d acceleration) {
        if (inverseMass != 0f) {
            gravity.scale(1f / inverseMass, acceleration);
        }
    }

    public Vector3d getGravity(Vector3d out) {
        out.set(gravity);
        return out;
    }

    public void setDamping(double lin_damping, double ang_damping) {
        linearDamping = MiscUtil.GEN_clamped(lin_damping, 0f, 1f);
        angularDamping = MiscUtil.GEN_clamped(ang_damping, 0f, 1f);
    }

    public double getLinearDamping() {
        return linearDamping;
    }

    public double getAngularDamping() {
        return angularDamping;
    }

    public double getLinearSleepingThreshold() {
        return linearSleepingThreshold;
    }

    public double getAngularSleepingThreshold() {
        return angularSleepingThreshold;
    }

    /**
     * Damps the velocity, using the given linearDamping and angularDamping.
     */
    public void applyDamping(double timeStep) {
        // On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
        // todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

        //#define USE_OLD_DAMPING_METHOD 1
        //#ifdef USE_OLD_DAMPING_METHOD
        //linearVelocity.scale(MiscUtil.GEN_clamped((1f - timeStep * linearDamping), 0f, 1f));
        //angularVelocity.scale(MiscUtil.GEN_clamped((1f - timeStep * angularDamping), 0f, 1f));
        //#else
        linearVelocity.scale(Math.pow(1f - linearDamping, timeStep));
        angularVelocity.scale(Math.pow(1f - angularDamping, timeStep));
        //#endif

        if (additionalDamping) {
            // Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
            // Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
            if ((angularVelocity.lengthSquared() < additionalAngularDampingThresholdSqr) &&
                    (linearVelocity.lengthSquared() < additionalLinearDampingThresholdSqr)) {
                angularVelocity.scale(additionalDampingFactor);
                linearVelocity.scale(additionalDampingFactor);
            }

            double speed = linearVelocity.length();
            if (speed < linearDamping) {
                double dampVel = 0.005f;
                if (speed > dampVel) {
                    Vector3d dir = new Vector3d(linearVelocity);
                    dir.normalize();
                    dir.scale(dampVel);
                    linearVelocity.sub(dir);
                } else {
                    linearVelocity.set(0f, 0f, 0f);
                }
            }

            double angSpeed = angularVelocity.length();
            if (angSpeed < angularDamping) {
                double angDampVel = 0.005f;
                if (angSpeed > angDampVel) {
                    Vector3d dir = new Vector3d(angularVelocity);
                    dir.normalize();
                    dir.scale(angDampVel);
                    angularVelocity.sub(dir);
                } else {
                    angularVelocity.set(0f, 0f, 0f);
                }
            }
        }
    }

    public void setMassProps(double mass, Vector3d inertia) {
        if (mass == 0f) {
            collisionFlags |= CollisionFlags.STATIC_OBJECT;
            inverseMass = 0f;
        } else {
            collisionFlags &= (~CollisionFlags.STATIC_OBJECT);
            inverseMass = 1f / mass;
        }

        invInertiaLocal.set(inertia.x != 0f ? 1f / inertia.x : 0f,
                inertia.y != 0f ? 1f / inertia.y : 0f,
                inertia.z != 0f ? 1f / inertia.z : 0f);
    }

    public double getInvMass() {
        return inverseMass;
    }

    public Matrix3d getInvInertiaTensorWorld(Matrix3d out) {
        out.set(invInertiaTensorWorld);
        return out;
    }

    public void integrateVelocities(double step) {
        if (isStaticOrKinematicObject()) {
            return;
        }

        linearVelocity.scaleAdd(inverseMass * step, totalForce, linearVelocity);
        Vector3d tmp = Stack.newVec(totalTorque);
        invInertiaTensorWorld.transform(tmp);
        angularVelocity.scaleAdd(step, tmp, angularVelocity);

        // clamp angular velocity. collision calculations will fail on higher angular velocities
        double angvel = angularVelocity.length();
        if (angvel * step > MAX_ANGVEL) {
            angularVelocity.scale((MAX_ANGVEL / step) / angvel);
        }
    }

    public void setCenterOfMassTransform(Transform xform) {
        if (isStaticOrKinematicObject()) {
            interpolationWorldTransform.set(worldTransform);
        } else {
            interpolationWorldTransform.set(xform);
        }
        getLinearVelocity(interpolationLinearVelocity);
        getAngularVelocity(interpolationAngularVelocity);
        worldTransform.set(xform);
        updateInertiaTensor();
    }

    public void applyCentralForce(Vector3d force) {
        totalForce.add(force);
    }

    public Vector3d getInvInertiaDiagLocal(Vector3d out) {
        out.set(invInertiaLocal);
        return out;
    }

    public void setInvInertiaDiagLocal(Vector3d diagInvInertia) {
        invInertiaLocal.set(diagInvInertia);
    }

    public void setSleepingThresholds(double linear, double angular) {
        linearSleepingThreshold = linear;
        angularSleepingThreshold = angular;
    }

    public void applyTorque(Vector3d torque) {
        totalTorque.add(torque);
    }

    public void applyForce(Vector3d force, Vector3d rel_pos) {
        applyCentralForce(force);

        Vector3d tmp = Stack.newVec();
        tmp.cross(rel_pos, force);
        tmp.scale(angularFactor);
        applyTorque(tmp);
    }

    public void applyCentralImpulse(Vector3d impulse) {
        linearVelocity.scaleAdd(inverseMass, impulse, linearVelocity);
    }

    @StaticAlloc
    public void applyTorqueImpulse(Vector3d torque) {
        Vector3d tmp = Stack.newVec(torque);
        invInertiaTensorWorld.transform(tmp);
        angularVelocity.add(tmp);
    }

    @StaticAlloc
    public void applyImpulse(Vector3d impulse, Vector3d rel_pos) {
        if (inverseMass != 0f) {
            applyCentralImpulse(impulse);
            if (angularFactor != 0f) {
                Vector3d tmp = Stack.newVec();
                tmp.cross(rel_pos, impulse);
                tmp.scale(angularFactor);
                applyTorqueImpulse(tmp);
            }
        }
    }

    /**
     * Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position.
     */
    public void internalApplyImpulse(Vector3d linearComponent, Vector3d angularComponent, double impulseMagnitude) {
        if (inverseMass != 0f) {
            linearVelocity.scaleAdd(impulseMagnitude, linearComponent, linearVelocity);
            if (angularFactor != 0f) {
                angularVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
            }
        }
    }

    public void clearForces() {
        totalForce.set(0f, 0f, 0f);
        totalTorque.set(0f, 0f, 0f);
    }

    public void updateInertiaTensor() {
        Matrix3d mat1 = Stack.newMat();
        MatrixUtil.scale(mat1, worldTransform.basis, invInertiaLocal);

        Matrix3d mat2 = Stack.newMat(worldTransform.basis);
        mat2.transpose();

        invInertiaTensorWorld.mul(mat1, mat2);

        Stack.subMat(2);
    }

    public Vector3d getCenterOfMassPosition(Vector3d out) {
        out.set(worldTransform.origin);
        return out;
    }

    public Quat4d getOrientation(Quat4d out) {
        MatrixUtil.getRotation(worldTransform.basis, out);
        return out;
    }

    public Transform getCenterOfMassTransform(Transform out) {
        out.set(worldTransform);
        return out;
    }

    public Matrix3d getCenterOfMassBasis(Matrix3d out) {
        out.set(worldTransform.basis);
        return out;
    }

    public Vector3d getLinearVelocity(Vector3d out) {
        out.set(linearVelocity);
        return out;
    }

    public double getLinearVelocityLengthSquared() {
        return linearVelocity.lengthSquared();
    }

    public Vector3d getAngularVelocity(Vector3d out) {
        out.set(angularVelocity);
        return out;
    }

    public double getAngularVelocityLengthSquared() {
        return angularVelocity.lengthSquared();
    }

    public void setLinearVelocity(Vector3d lin_vel) {
        assert (collisionFlags != CollisionFlags.STATIC_OBJECT);
        linearVelocity.set(lin_vel);
    }

    public void setAngularVelocity(Vector3d ang_vel) {
        assert (collisionFlags != CollisionFlags.STATIC_OBJECT);
        angularVelocity.set(ang_vel);
    }

    public Vector3d getVelocityInLocalPoint(Vector3d rel_pos, Vector3d out) {
        // we also calculate lin/ang velocity for kinematic objects
        out.cross(angularVelocity, rel_pos);
        out.add(linearVelocity);
        return out;

        //for kinematic objects, we could also use use:
        //		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
    }

    public void translate(Vector3d v) {
        worldTransform.origin.add(v);
    }

    public void getAabb(Vector3d aabbMin, Vector3d aabbMax) {
        getCollisionShape().getAabb(worldTransform, aabbMin, aabbMax);
    }

    public double computeImpulseDenominator(Vector3d pos, Vector3d normal) {

        int v3 = Stack.getVecPosition();
        int m3 = Stack.getMatPosition();

        Vector3d r0 = Stack.newVec();
        r0.sub(pos, getCenterOfMassPosition(Stack.newVec()));

        Vector3d c0 = Stack.newVec();
        c0.cross(r0, normal);

        Vector3d tmp = Stack.newVec();
        MatrixUtil.transposeTransform(tmp, c0, getInvInertiaTensorWorld(Stack.newMat()));

        Vector3d vec = Stack.newVec();
        vec.cross(tmp, r0);

        Stack.resetVec(v3);
        Stack.resetMat(m3);

        return inverseMass + normal.dot(vec);

    }

    public double computeAngularImpulseDenominator(Vector3d axis) {
        Vector3d vec = new Vector3d();
        MatrixUtil.transposeTransform(vec, axis, getInvInertiaTensorWorld(new Matrix3d()));
        return axis.dot(vec);
    }

    public void updateDeactivation(double timeStep) {
        if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION)) {
            return;
        }

        if (getLinearVelocityLengthSquared() < linearSleepingThreshold * linearSleepingThreshold &&
                getAngularVelocityLengthSquared() < angularSleepingThreshold * angularSleepingThreshold) {
            deactivationTime += timeStep;
        } else {
            deactivationTime = 0f;
            setActivationState(0);
        }
    }

    public boolean wantsSleeping() {
        if (getActivationState() == DISABLE_DEACTIVATION) {
            return false;
        }

        // disable deactivation
        if (BulletGlobals.isDeactivationDisabled() || (BulletGlobals.getDeactivationTime() == 0f)) {
            return false;
        }

        if ((getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION)) {
            return true;
        }

        if (deactivationTime > BulletGlobals.getDeactivationTime()) {
            return true;
        }
        return false;
    }

    public BroadphaseProxy getBroadphaseProxy() {
        return broadphaseHandle;
    }

    public void setNewBroadphaseProxy(BroadphaseProxy broadphaseProxy) {
        this.broadphaseHandle = broadphaseProxy;
    }

    public MotionState getMotionState() {
        return optionalMotionState;
    }

    public void setMotionState(MotionState motionState) {
        this.optionalMotionState = motionState;
        if (optionalMotionState != null) {
            motionState.getWorldTransform(worldTransform);
        }
    }

    public void setAngularFactor(double angFac) {
        angularFactor = angFac;
    }

    public double getAngularFactor() {
        return angularFactor;
    }

    /**
     * Is this rigidbody added to a CollisionWorld/DynamicsWorld/Broadphase?
     */
    public boolean isInWorld() {
        return (getBroadphaseProxy() != null);
    }

    @Override
    public boolean checkCollideWithOverride(CollisionObject co) {
        // TODO: change to cast
        RigidBody otherRb = RigidBody.upcast(co);
        if (otherRb == null) {
            return true;
        }

        for (int i = 0; i < constraintRefs.size(); ++i) {
            TypedConstraint c = constraintRefs.getQuick(i);
            if (c.getRigidBodyA() == otherRb || c.getRigidBodyB() == otherRb) {
                return false;
            }
        }

        return true;
    }

    public void addConstraintRef(TypedConstraint c) {
        int index = constraintRefs.indexOf(c);
        if (index == -1) {
            constraintRefs.add(c);
        }

        checkCollideWith = true;
    }

    public void removeConstraintRef(TypedConstraint c) {
        constraintRefs.remove(c);
        checkCollideWith = (constraintRefs.size() > 0);
    }

    public TypedConstraint getConstraintRef(int index) {
        return constraintRefs.getQuick(index);
    }

    public int getNumConstraintRefs() {
        return constraintRefs.size();
    }

}
