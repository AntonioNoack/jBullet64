package com.bulletphysics;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.vehicle.DefaultVehicleRaycaster;
import com.bulletphysics.dynamics.vehicle.RaycastVehicle;
import com.bulletphysics.dynamics.vehicle.VehicleTuning;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import static com.bulletphysics.StackOfBoxesTest.createRigidBody;
import static com.bulletphysics.StackOfBoxesTest.createWorld;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class VehicleTest {
    private RaycastVehicle createVehicle(DiscreteDynamicsWorld world, Vector3f startPos) {
        CollisionShape chassisShape = new BoxShape(new Vector3d(1f, 0.5f, 2f)); // Simple box car

        RigidBody chassis = createRigidBody(800f, startPos, chassisShape); // Heavy for stability
        world.addRigidBody(chassis);

        VehicleTuning tuning = new VehicleTuning();
        tuning.suspensionStiffness = 20f;
        tuning.suspensionDamping = 2.3f;
        tuning.suspensionCompression = 4.4f;
        tuning.frictionSlip = 1000f;
        tuning.maxSuspensionTravelCm = 500f;

        DefaultVehicleRaycaster raycaster = new DefaultVehicleRaycaster(world);
        RaycastVehicle vehicle = new RaycastVehicle(tuning, chassis, raycaster);
        chassis.setActivationState(CollisionObject.DISABLE_DEACTIVATION);

        world.addVehicle(vehicle);

        // Add 4 wheels
        Vector3d wheelDirection = new Vector3d(0, -1, 0);
        Vector3d wheelAxle = new Vector3d(-1, 0, 0);
        float suspensionRestLength = 0.6f;
        float wheelRadius = 0.5f;
        boolean isFront = true;

        // Positions relative to chassis
        vehicle.addWheel(new Vector3d(1f, -0.5f, 2f), wheelDirection, wheelAxle, suspensionRestLength, wheelRadius, tuning, isFront);
        vehicle.addWheel(new Vector3d(-1f, -0.5f, 2f), wheelDirection, wheelAxle, suspensionRestLength, wheelRadius, tuning, isFront);
        vehicle.addWheel(new Vector3d(1f, -0.5f, -2f), wheelDirection, wheelAxle, suspensionRestLength, wheelRadius, tuning, false);
        vehicle.addWheel(new Vector3d(-1f, -0.5f, -2f), wheelDirection, wheelAxle, suspensionRestLength, wheelRadius, tuning, false);

        return vehicle;
    }

    private Vector3d normalize(Vector3d d) {
        d.normalize();
        return d;
    }

    @Test
    public void testVehicleBehavior() {
        // Step 1: Flat ground test
        DiscreteDynamicsWorld world = createWorld();
        createGroundPlane(new Vector3d(0, 1, 0), world); // Flat plane
        RaycastVehicle vehicle = createVehicle(world, new Vector3f(0, 2, 0));

        simulate(world, 120);
        Vector3d flatPos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(flatPos);
        assertTrue(flatPos.y < 1.5f && flatPos.y > 0.5f, "Vehicle should rest on ground");

        // Step 2: Hill test (no engine force)
        world = createWorld();
        createGroundPlane(normalize(new Vector3d(0, 1, -0.5f)), world); // Inclined
        vehicle = createVehicle(world, new Vector3f(0, 2, 0));

        simulate(world, 240); // 4 seconds
        Vector3d slopePos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(slopePos);
        assertTrue(slopePos.z < -1f, "Vehicle should have rolled downhill");

        // Step 3: Driving test
        world = createWorld();
        createGroundPlane(new Vector3d(0, 1, 0), world); // Flat
        vehicle = createVehicle(world, new Vector3f(0, 2, 0));
        applyEngineForce(vehicle, 800f); // Drive forward

        simulate(world, 240);
        Vector3d drivePos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(drivePos);
        assertTrue(drivePos.z > 1f, "Vehicle should drive forward");

        // Step 4: Driving downhill
        world = createWorld();
        createGroundPlane(normalize(new Vector3d(0, 1, 0.2f)), world); // Mild hill
        vehicle = createVehicle(world, new Vector3f(0, 2, 0));
        applyEngineForce(vehicle, 800f);

        simulate(world, 240);
        Vector3d downPos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(downPos);
        assertTrue(downPos.z > 2f, "Vehicle should drive faster downhill");

        // Step 5: Turning
        world = createWorld();
        createGroundPlane(new Vector3d(0, 1, 0), world); // Flat again
        vehicle = createVehicle(world, new Vector3f(0, 2, 0));
        applyEngineForce(vehicle, 800f);
        applySteering(vehicle, 0.3f); // Turn wheels slightly

        simulate(world, 240);
        Vector3d turnPos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(turnPos);
        assertTrue(Math.abs(turnPos.x) > 0.5f, "Vehicle should have turned");
    }

    @Test
    public void testVehicleBrakingDownhill() {
        DiscreteDynamicsWorld world = createWorld();

        // Create downhill plane (slope in +Z)
        createGroundPlane(normalize(new Vector3d(0, 1, 0.2f)), world);

        // Create vehicle
        RaycastVehicle vehicle = createVehicle(world, new Vector3f(0, 2, 0));

        // Phase 1: Drive downhill for 2 seconds
        applyEngineForce(vehicle, 800f);
        simulate(world, 120); // 2 seconds
        Vector3d preBrakePos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(preBrakePos);

        System.out.println("Distance traveled during driving phase: deltaZ = " + preBrakePos.z);

        // Phase 2: Apply brakes, no more engine force
        applyEngineForce(vehicle, 0f);
        applyBrakes(vehicle, 200f); // Strong braking force
        simulate(world, 120); // 2 more seconds
        Vector3d postBrakePos = new Vector3d();
        vehicle.getRigidBody().getCenterOfMassPosition(postBrakePos);

        double deltaZ = postBrakePos.z - preBrakePos.z;

        System.out.println("Distance traveled during brake phase: deltaZ = " + deltaZ);

        // Assert that braking significantly reduced forward motion
        assertTrue(deltaZ < 0.7f, "Vehicle should slow down when braking downhill");
    }

    @SuppressWarnings("SameParameterValue")
    private void applyBrakes(RaycastVehicle vehicle, float brakeForce) {
        for (int i = 0; i < vehicle.getNumWheels(); i++) {
            vehicle.setBrake(brakeForce, i);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void applyEngineForce(RaycastVehicle vehicle, float force) {
        for (int i = 0; i < vehicle.getNumWheels(); i++) {
            vehicle.applyEngineForce(force, i);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void applySteering(RaycastVehicle vehicle, float steering) {
        // Only front wheels
        vehicle.setSteeringValue(steering, 0);
        vehicle.setSteeringValue(steering, 1);
    }

    private void simulate(DiscreteDynamicsWorld world, int steps) {
        for (int i = 0; i < steps; i++) {
            world.stepSimulation(1f / 60f, 10);
            Stack.reset(false);
        }
    }

    private void createGroundPlane(Vector3d normal, DiscreteDynamicsWorld world) {
        CollisionShape planeShape = new StaticPlaneShape(normal, (float) 0);
        RigidBody ground = createRigidBody(0f, new Vector3f(0, 0, 0), planeShape);
        world.addRigidBody(ground);
    }
}
