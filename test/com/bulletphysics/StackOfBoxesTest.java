package com.bulletphysics;

import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SliderConstraint;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class StackOfBoxesTest {

    @Test
    public void testStackDoesNotFall() {
        DiscreteDynamicsWorld dynamicsWorld = createWorld();
        createGround(dynamicsWorld);

        RigidBody[] boxes = createBoxTower(dynamicsWorld, 0f);
        runSimulation(dynamicsWorld);

        // Validate stack has not fallen
        for (int i = 0; i < boxes.length; i++) {
            Transform trans = new Transform();
            boxes[i].getMotionState().getWorldTransform(trans);
            double y = trans.origin.y;
            System.out.println(trans.origin);
            assertTrue(Math.abs(trans.origin.x) < 0.5f, "Box " + i + " fell sideways on X axis");
            assertTrue(Math.abs(trans.origin.z) < 0.5f, "Box " + i + " fell sideways on Z axis");
            assertTrue(y > 0.1f, "Box " + i + " dropped too low");
        }
    }

    @Test
    public void testStackFallsOverWhenOffset() {
        DiscreteDynamicsWorld dynamicsWorld = createWorld();
        createGround(dynamicsWorld);

        RigidBody[] boxes = createBoxTower(dynamicsWorld, 0.6f);
        runSimulation(dynamicsWorld);

        // Check that the top box has significantly deviated horizontally (i.e., tower has fallen)
        Transform topBoxTransform = new Transform();
        boxes[boxes.length - 1].getMotionState().getWorldTransform(topBoxTransform);
        double finalX = topBoxTransform.origin.x;
        double finalZ = topBoxTransform.origin.z;

        // Threshold assumes a fall if final X or Z is way off from initial offset
        boolean fellOver = Math.abs(finalX) > 3.0f || Math.abs(finalZ) > 1.0f;
        assertTrue(fellOver, "The stack did not fall over as expected. Final X: " + finalX + ", Z: " + finalZ);
    }

    @Test
    public void testHeavySphereKnocksOverTower() {
        // Physics setup
        DiscreteDynamicsWorld world = createWorld();
        createGround(world);

        // Tower of boxes
        RigidBody[] boxes = createBoxTower(world, 0f);
        float boxSize = 1f;

        // Sphere shape and setup
        float sphereRadius = 0.5f;
        float sphereMass = 50f; // Heavy sphere
        CollisionShape sphereShape = new SphereShape(sphereRadius);

        Transform sphereTransform = new Transform();
        sphereTransform.setIdentity();
        sphereTransform.origin.set(-5f, boxSize * 2 * boxes.length - 1f, 0); // at height of top box

        RigidBody sphereBody = createRigidBody(sphereMass, sphereTransform, sphereShape);
        sphereBody.setFriction(0.0f);
        sphereBody.setRestitution(0.2f); // some energy loss on impact
        world.addRigidBody(sphereBody);

        Transform frameInWorld = new Transform();
        frameInWorld.setIdentity();
        frameInWorld.origin.set(sphereTransform.origin); // same as sphere's origin


        // Static body representing the world (for slider reference)
        RigidBody staticRail = createRigidBody(0f, new Transform(), new BoxShape(new Vector3d(0.1f, 0.1f, 0.1f)));
        world.addRigidBody(staticRail);

        // Frame in sphere's local space (starts at origin)
        Transform frameInA = new Transform();
        frameInA.setIdentity();
        frameInA.origin.set(0f, 0f, 0f);

        // Frame in static body's local space — defines the rail's origin and direction
        Transform frameInB = new Transform();
        frameInB.setIdentity();
        frameInB.origin.set(sphereTransform.origin); // matches sphere start pos
        // You can also set frameInB.basis to rotate if you want motion along a different axis

        SliderConstraint slider = new SliderConstraint(sphereBody, staticRail, frameInA, frameInB, true);
        slider.setLowerLinLimit(-10f);
        slider.setUpperLinLimit(10f);
        slider.setLowerAngLimit(0f);
        slider.setUpperAngLimit(0f);
        world.addConstraint(slider);

        // Apply initial velocity to slide the sphere along X toward the tower
        sphereBody.setLinearVelocity(new Vector3d(10f, 0f, 0f)); // Move right

        runSimulation(world);

        // Verify the tower has fallen (by checking top box's horizontal deviation)
        Transform topTransform = new Transform();
        boxes[boxes.length - 1].getMotionState().getWorldTransform(topTransform);
        double dx = Math.abs(topTransform.origin.x);
        double dz = Math.abs(topTransform.origin.z);

        boolean towerFell = dx > 1.0f || dz > 1.0f;
        assertTrue(towerFell, "Tower did not fall after being hit. X offset: " + dx + ", Z offset: " + dz);
    }

    private void runSimulation(DiscreteDynamicsWorld dynamicsWorld) {
        // Run the simulation
        float timeStep = 1f / 60f;
        int maxSubSteps = 10;
        int steps = 240; // 4 seconds of simulation
        for (int i = 0; i < steps; i++) {
            dynamicsWorld.stepSimulation(timeStep, maxSubSteps);
        }
    }

    private RigidBody[] createBoxTower(DynamicsWorld dynamicsWorld, float xOffsetPerBox) {
        // Boxes
        float boxSize = 1f;
        int boxCount = 5;
        RigidBody[] boxes = new RigidBody[boxCount];
        CollisionShape boxShape = new BoxShape(new Vector3d(boxSize, boxSize, boxSize));

        for (int i = 0; i < boxCount; i++) {
            Transform boxTransform = new Transform();
            boxTransform.setIdentity();
            float y = (2 * boxSize * i) + boxSize;
            float x = xOffsetPerBox * i; // cumulative horizontal offset
            boxTransform.origin.set(x, y, 0);
            boxes[i] = createRigidBody(1f, boxTransform, boxShape);
            dynamicsWorld.addRigidBody(boxes[i]);
        }
        return boxes;
    }

    private void createGround(DiscreteDynamicsWorld dynamicsWorld) {
        // Ground plane
        CollisionShape groundShape = new StaticPlaneShape(new Vector3d(0, 1, 0), 1);
        Transform groundTransform = new Transform();
        groundTransform.setIdentity();
        groundTransform.origin.set(0, -1, 0);
        RigidBody groundBody = createRigidBody(0, groundTransform, groundShape);
        dynamicsWorld.addRigidBody(groundBody);
    }

    private DiscreteDynamicsWorld createWorld() {
        // Physics setup
        DefaultCollisionConfiguration collisionConfig = new DefaultCollisionConfiguration();
        CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfig);
        Vector3d worldAabbMin = new Vector3d(-1000, -1000, -1000);
        Vector3d worldAabbMax = new Vector3d(1000, 1000, 1000);
        AxisSweep3 broadphase = new AxisSweep3(worldAabbMin, worldAabbMax);
        SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
        DiscreteDynamicsWorld dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);

        dynamicsWorld.setGravity(new Vector3d(0f, -10f, 0f));
        return dynamicsWorld;
    }

    private RigidBody createRigidBody(float mass, Transform transform, CollisionShape shape) {
        Vector3d localInertia = new Vector3d(0, 0, 0);
        if (mass != 0f) {
            shape.calculateLocalInertia(mass, localInertia);
        }

        DefaultMotionState motionState = new DefaultMotionState(transform);
        RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, motionState, shape, localInertia);
        return new RigidBody(rbInfo);
    }
}
