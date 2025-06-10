package com.bulletphysics;

import com.bulletphysics.collision.broadphase.CollisionFilterGroups;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.GhostPairCallback;
import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import static com.bulletphysics.StackOfBoxesTest.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class GhostObjectTest {

    @Test
    public void testGhostObjectOverlapDetection() {
        DiscreteDynamicsWorld world = createWorld();
        world.getBroadphase().getOverlappingPairCache().setInternalGhostPairCallback(new GhostPairCallback());
        createGround(world);

        // Ghost object region (box shape)
        BoxShape ghostShape = new BoxShape(new Vector3d(1f, 0.5f, 1f));
        PairCachingGhostObject ghost = new PairCachingGhostObject();
        ghost.setCollisionShape(ghostShape);
        Transform ghostTransform = new Transform();
        ghostTransform.setIdentity();
        ghostTransform.origin.set(0, 3.0f, 0); // floating trigger region
        ghost.setWorldTransform(ghostTransform);
        ghost.setCollisionFlags(CollisionFlags.NO_CONTACT_RESPONSE); // no physics response

        // Must register ghost object in collision world
        world.addCollisionObject(ghost, CollisionFilterGroups.SENSOR_TRIGGER, (short) -1);

        // Falling dynamic sphere
        CollisionShape sphereShape = new SphereShape(0.25f);
        RigidBody sphere = createRigidBody(1f, new Vector3f(0, 5, 0), sphereShape);
        world.addRigidBody(sphere);

        boolean entered = false;
        boolean exited = false;
        boolean wasInside = false;

        // Simulate
        float timeStep = 1f / 60f;
        int steps = 70;

        for (int i = 0; i < steps; i++) {
            world.stepSimulation(timeStep);

            int numOverlaps = ghost.getNumOverlappingObjects();
            boolean inside = false;

            for (int j = 0; j < numOverlaps; j++) {
                if (ghost.getOverlappingObject(j) == sphere) {
                    inside = true;
                    break;
                }
            }

            if (inside && !wasInside) {
                entered = true;
            } else if (!inside && wasInside) {
                exited = true;
            }

            wasInside = inside;
        }

        // Assertions
        assertTrue(entered, "Sphere should have entered ghost zone.");
        assertTrue(exited, "Sphere should have exited ghost zone.");
    }

}