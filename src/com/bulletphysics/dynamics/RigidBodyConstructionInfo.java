package com.bulletphysics.dynamics;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3d;

/**
 * RigidBodyConstructionInfo provides information to create a rigid body.<p>
 * <p>
 * Setting mass to zero creates a fixed (non-dynamic) rigid body. For dynamic objects,
 * you can use the collision shape to approximate the local inertia tensor, otherwise
 * use the zero vector (default argument).<p>
 * <p>
 * You can use {@link MotionState} to synchronize the world transform
 * between physics and graphics objects. And if the motion state is provided, the rigid
 * body will initialize its initial world transform from the motion state,
 * {@link #startWorldTransform startWorldTransform} is only used when you don't provide
 * a motion state.
 *
 * @author jezek2
 */
public class RigidBodyConstructionInfo {

    public double mass;

    /**
     * When a motionState is provided, the rigid body will initialize its world transform
     * from the motion state. In this case, startWorldTransform is ignored.
     */
    public MotionState motionState;
    public final Transform startWorldTransform = new Transform();

    public CollisionShape collisionShape;
    public final Vector3d localInertia = new Vector3d();
    public double linearDamping = 0.0;
    public double angularDamping = 0.0;

    /**
     * Best simulation results when friction is non-zero.
     */
    public double friction = 0.5;
    /**
     * Best simulation results using zero restitution.
     */
    public double restitution = 0.0;

    public double linearSleepingThreshold = 0.8;
    public double angularSleepingThreshold = 1.0;

    /**
     * Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
     * Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics
     * system has improved, this should become obsolete.
     */
    public boolean additionalDamping = false;
    public double additionalDampingFactor = 0.005;
    public double additionalLinearDampingThresholdSqr = 0.01;
    public double additionalAngularDampingThresholdSqr = 0.01;
    public double additionalAngularDampingFactor = 0.01;

    @SuppressWarnings("unused")
    public RigidBodyConstructionInfo(double mass, MotionState motionState, CollisionShape collisionShape) {
        this(mass, motionState, collisionShape, new Vector3d(0.0, 0.0, 0.0));
    }

    public RigidBodyConstructionInfo(double mass, MotionState motionState, CollisionShape collisionShape, Vector3d localInertia) {
        this.mass = mass;
        this.motionState = motionState;
        this.collisionShape = collisionShape;
        this.localInertia.set(localInertia);

        startWorldTransform.setIdentity();
    }

}
