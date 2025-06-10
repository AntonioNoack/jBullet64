package com.bulletphysics;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.Point2PointConstraint;
import com.bulletphysics.dynamics.constraintsolver.SliderConstraint;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import static com.bulletphysics.StackOfBoxesTest.createWorld;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ConstraintTest {
    @Test
    public void testPoint2PointConstraint() {
        DiscreteDynamicsWorld world = createWorld();

        // Two bodies
        RigidBody bodyA = createDynamicBox(new Vector3f(-1, 5, 0), 1f);
        RigidBody bodyB = createDynamicBox(new Vector3f(1, 5, 0), 1f);
        world.addRigidBody(bodyA);
        world.addRigidBody(bodyB);

        // Link via ball joint
        Point2PointConstraint p2p = new Point2PointConstraint(bodyA, bodyB,
                new Vector3d(0.5f, 0, 0), new Vector3d(-0.5f, 0, 0));
        world.addConstraint(p2p, true);

        simulate(world, 240);

        Vector3d delta = new Vector3d();
        delta.sub(bodyA.getCenterOfMassPosition(new Vector3d()), bodyB.getCenterOfMassPosition(new Vector3d()));
        double dist = delta.length();
        System.out.println("P2P distance: " + dist);
        assertTrue(dist < 1.2f, "Bodies should remain close due to point2point constraint");
    }

    @Test
    public void testHingeConstraint() {
        DiscreteDynamicsWorld world = createWorld();

        RigidBody base = createStaticBox(new Vector3f(0, 5, 0));
        RigidBody bar = createDynamicBox(new Vector3f(0.1f, 4, 0), 1f);
        world.addRigidBody(base);
        world.addRigidBody(bar);

        Transform pivotInA = new Transform();
        pivotInA.setIdentity();
        pivotInA.origin.set(0, -0.5f, 0);
        Transform pivotInB = new Transform();
        pivotInB.setIdentity();
        pivotInB.origin.set(0, 0.5f, 0);

        HingeConstraint hinge = new HingeConstraint(base, bar, pivotInA, pivotInB);
        world.addConstraint(hinge, true);

        simulate(world, 240);

        double angle = hinge.getHingeAngle();
        System.out.println("Hinge angle: " + angle);
        assertTrue(Math.abs(angle) > 0.01f, "Hinge should allow rotation");
    }

    @Test
    public void testSliderConstraint() {
        DiscreteDynamicsWorld world = createWorld();

        RigidBody base = createStaticBox(new Vector3f(0, 0, 0));
        RigidBody slider = createDynamicBox(new Vector3f(0, 0, 0.5f), 1f);
        world.addRigidBody(base);
        world.addRigidBody(slider);

        Transform frameInA = new Transform();
        frameInA.setIdentity();
        frameInA.origin.set(0, 0, 0);
        Transform frameInB = new Transform();
        frameInB.setIdentity();
        frameInB.origin.set(0, 0, 0);

        SliderConstraint sliderConstraint = new SliderConstraint(base, slider, frameInA, frameInB, true);
        world.addConstraint(sliderConstraint, true);

        slider.applyCentralForce(new Vector3d(20, 20, 20)); // Slide forward

        simulate(world, 240);

        Vector3d pos = slider.getCenterOfMassPosition(new Vector3d());
        assertTrue(Math.abs(pos.y) < 1e-9);
        assertTrue(Math.abs(pos.z) < 1e-9);
        assertTrue(pos.x > 0.5f, "Slider should move along Z axis");
    }

    @Test
    public void testGeneric6DofConstraint() {
        DiscreteDynamicsWorld world = createWorld();

        RigidBody base = createStaticBox(new Vector3f(0, 0, 0));
        RigidBody body = createDynamicBox(new Vector3f(0, 0, 0.5f), 1f);
        world.addRigidBody(base);
        world.addRigidBody(body);

        Transform frameInA = new Transform();
        frameInA.setIdentity();
        Transform frameInB = new Transform();
        frameInB.setIdentity();

        Generic6DofConstraint dof = new Generic6DofConstraint(base, body, frameInA, frameInB, true);
        dof.setLinearLowerLimit(new Vector3d(-1, 0, 0));
        dof.setLinearUpperLimit(new Vector3d(1, 0, 0)); // Only X axis movement allowed
        world.addConstraint(dof, true);

        body.applyCentralForce(new Vector3d(20, 0, 20));

        simulate(world, 240);

        Vector3d pos = body.getCenterOfMassPosition(new Vector3d());
        System.out.println("6DoF position: " + pos);
        assertTrue(Math.abs(pos.z) < 0.1f, "Movement in Z should be restricted");
        assertTrue(pos.x > 0.5f, "Movement in X should be allowed");
    }

    @Test
    public void testConstraintBreaking() {
        DiscreteDynamicsWorld world = createWorld();

        // Create two dynamic bodies
        RigidBody bodyA = createDynamicBox(new Vector3f(0, 5, 0), 1f);
        RigidBody bodyB = createDynamicBox(new Vector3f(0, 4, 0), 1f);
        world.addRigidBody(bodyA);
        world.addRigidBody(bodyB);

        // Attach with a point2point (ball-socket) constraint
        Point2PointConstraint constraint = new Point2PointConstraint(bodyA, bodyB,
                new Vector3d(0, -0.5f, 0), new Vector3d(0, 0.5f, 0));
        constraint.setBreakingImpulseThreshold(5f); // Very low threshold
        world.addConstraint(constraint, true);

        // Apply strong impulse to break it
        bodyB.applyCentralImpulse(new Vector3d(50, 0, 0));

        simulate(world, 60);

        // Check whether constraint has been removed
        boolean broken = world.getNumConstraints() == 0;
        System.out.println("Constraint broken: " + broken);

        assertTrue(broken, "The constraint should break under strong impulse.");
        assertTrue(constraint.isBroken());
    }

    @Test
    public void testHingeMotorWithLimit() {
        DiscreteDynamicsWorld world = createWorld();

        // Static anchor and rotating bar
        RigidBody base = createStaticBox(new Vector3f(0, 5, 0));
        RigidBody bar = createDynamicBox(new Vector3f(1, 5, 0), 1f);
        world.addRigidBody(base);
        world.addRigidBody(bar);

        Transform pivotInA = new Transform();
        pivotInA.setIdentity();
        pivotInA.origin.set(0.5f, 0, 0);

        Transform pivotInB = new Transform();
        pivotInB.setIdentity();
        pivotInB.origin.set(-0.5f, 0, 0);

        HingeConstraint hinge = new HingeConstraint(base, bar, pivotInA, pivotInB);
        hinge.setLimit(-Math.PI / 4, Math.PI / 4); // ±45°
        hinge.enableAngularMotor(true, 2.0f, 0.1f); // velocity, max impulse

        world.addConstraint(hinge, true);

        System.out.println("Limit: " + Math.PI / 4);
        for (int i = 0; i < 10; i++) {
            System.out.println("hinge angle: " + hinge.getHingeAngle());
            simulate(world, 18);
        }

        double angle = hinge.getHingeAngle();
        System.out.println("Final hinge angle: " + angle);

        assertTrue(Math.abs(angle) <= Math.PI / 4 + 0.05f, "Hinge angle should not exceed limit.");
    }

    private RigidBody createDynamicBox(Vector3f pos, float mass) {
        CollisionShape shape = new BoxShape(new Vector3d(0.5f, 0.5f, 0.5f));
        Transform t = new Transform();
        t.setIdentity();
        t.origin.set(pos);
        DefaultMotionState motionState = new DefaultMotionState(t);
        Vector3d inertia = new Vector3d();
        shape.calculateLocalInertia(mass, inertia);
        RigidBodyConstructionInfo info = new RigidBodyConstructionInfo(mass, motionState, shape, inertia);
        return new RigidBody(info);
    }

    private RigidBody createStaticBox(Vector3f pos) {
        return createDynamicBox(pos, 0f);
    }

    private void simulate(DynamicsWorld world, int steps) {
        float timeStep = 1f / 60f;
        for (int i = 0; i < steps; i++) {
            world.stepSimulation(timeStep);
        }
    }
}
