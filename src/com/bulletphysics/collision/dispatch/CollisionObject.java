package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3d;

/**
 * CollisionObject can be used to manage collision detection objects.
 * It maintains all information that is needed for a collision detection: {@link CollisionShape},
 * {@link Transform} and {@link BroadphaseProxy AABB proxy}. It can be added to {@link CollisionWorld}.
 *
 * @author jezek2
 */
public class CollisionObject {

    // island management, m_activationState1
    public static final int ACTIVE_TAG = 1;
    public static final int ISLAND_SLEEPING = 2;
    public static final int WANTS_DEACTIVATION = 3;
    public static final int DISABLE_DEACTIVATION = 4;
    public static final int DISABLE_SIMULATION = 5;
    protected Transform worldTransform = new Transform();

    /// m_interpolationWorldTransform is used for CCD and interpolation
    /// it can be either previous or future (predicted) transform
    protected final Transform interpolationWorldTransform = new Transform();

    /**
     * those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities)
     * without destroying the continuous interpolated motion (which uses this interpolation velocities)
     */
    protected final Vector3d interpolationLinearVelocity = new Vector3d();
    protected final Vector3d interpolationAngularVelocity = new Vector3d();
    protected BroadphaseProxy broadphaseHandle;
    protected CollisionShape collisionShape;

    /**
     * rootCollisionShape is temporarily used to store the original collision shape
     * The collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
     * If it is null, the collisionShape is not temporarily replaced.
     */
    protected CollisionShape rootCollisionShape;

    public int collisionFlags;
    public int islandTag;
    public int companionId;
    protected int activationState;
    public double deactivationTime;
    public double friction;
    public double restitution;

    /**
     * users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer
     * */
    public Object userObjectPointer;

    /**
     * time of impact calculation
     * */
    public double hitFraction;

    /**
     * Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
     */
    public double ccdSweptSphereRadius;

    /**
     * Don't do continuous collision detection if the motion (in one step) is less then ccdMotionThreshold
     */
    public double ccdMotionThreshold = 0.0;

    /**
     * If some object should have elaborate collision filtering by subclasses
     */
    protected boolean checkCollideWith;

    public CollisionObject() {
        this.collisionFlags = CollisionFlags.STATIC_OBJECT;
        this.islandTag = -1;
        this.companionId = -1;
        this.activationState = 1;
        this.friction = 0.5;
        this.hitFraction = 1.0;
    }

    public boolean checkCollideWithOverride(CollisionObject co) {
        return true;
    }

    public boolean mergesSimulationIslands() {
        ///static objects, kinematic and object without contact response don't merge islands
        return ((collisionFlags & (CollisionFlags.STATIC_OBJECT | CollisionFlags.KINEMATIC_OBJECT | CollisionFlags.NO_CONTACT_RESPONSE)) == 0);
    }

    public boolean isStaticObject() {
        return (collisionFlags & CollisionFlags.STATIC_OBJECT) != 0;
    }

    public boolean isKinematicObject() {
        return (collisionFlags & CollisionFlags.KINEMATIC_OBJECT) != 0;
    }

    public boolean isStaticOrKinematicObject() {
        return (collisionFlags & (CollisionFlags.KINEMATIC_OBJECT | CollisionFlags.STATIC_OBJECT)) != 0;
    }

    public boolean hasContactResponse() {
        return (collisionFlags & CollisionFlags.NO_CONTACT_RESPONSE) == 0;
    }

    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    public void setCollisionShape(CollisionShape collisionShape) {
        this.collisionShape = collisionShape;
        this.rootCollisionShape = collisionShape;
    }

    @SuppressWarnings("unused")
    public CollisionShape getRootCollisionShape() {
        return rootCollisionShape;
    }

    /**
     * Avoid using this internal API call.
     * internalSetTemporaryCollisionShape is used to temporary replace the actual collision shape by a child collision shape.
     */
    public void internalSetTemporaryCollisionShape(CollisionShape collisionShape) {
        this.collisionShape = collisionShape;
    }

    public int getActivationState() {
        return activationState;
    }

    public void setActivationState(int newState) {
        if ((activationState != DISABLE_DEACTIVATION) && (activationState != DISABLE_SIMULATION)) {
            this.activationState = newState;
        }
    }

    @SuppressWarnings("unused")
    public void forceActivationState(int newState) {
        this.activationState = newState;
    }

    public void activate() {
        activate(false);
    }

    public void activate(boolean forceActivation) {
        if (forceActivation || (collisionFlags & (CollisionFlags.STATIC_OBJECT | CollisionFlags.KINEMATIC_OBJECT)) == 0) {
            setActivationState(ACTIVE_TAG);
            deactivationTime = 0.0;
        }
    }

    public boolean isActive() {
        return ((getActivationState() != ISLAND_SLEEPING) && (getActivationState() != DISABLE_SIMULATION));
    }

    public Transform getWorldTransform(Transform out) {
        out.set(worldTransform);
        return out;
    }

    public void setWorldTransform(Transform worldTransform) {
        this.worldTransform.set(worldTransform);
    }

    public BroadphaseProxy getBroadphaseHandle() {
        return broadphaseHandle;
    }

    public void setBroadphaseHandle(BroadphaseProxy broadphaseHandle) {
        this.broadphaseHandle = broadphaseHandle;
    }

    public Transform getInterpolationWorldTransform(Transform out) {
        out.set(interpolationWorldTransform);
        return out;
    }

    public void setInterpolationWorldTransform(Transform interpolationWorldTransform) {
        this.interpolationWorldTransform.set(interpolationWorldTransform);
    }

    @SuppressWarnings("unused")
    public void setInterpolationLinearVelocity(Vector3d linvel) {
        interpolationLinearVelocity.set(linvel);
    }

    @SuppressWarnings("unused")
    public void setInterpolationAngularVelocity(Vector3d angvel) {
        interpolationAngularVelocity.set(angvel);
    }

    public Vector3d getInterpolationLinearVelocity(Vector3d out) {
        out.set(interpolationLinearVelocity);
        return out;
    }

    public Vector3d getInterpolationAngularVelocity(Vector3d out) {
        out.set(interpolationAngularVelocity);
        return out;
    }

    public double getCcdSquareMotionThreshold() {
        return ccdMotionThreshold * ccdMotionThreshold;
    }

    public boolean checkCollideWith(CollisionObject co) {
        if (checkCollideWith) {
            return checkCollideWithOverride(co);
        }
        return true;
    }
}
