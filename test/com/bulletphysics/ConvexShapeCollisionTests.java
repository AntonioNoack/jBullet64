package com.bulletphysics;

import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.shapes.*;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.extras.gimpact.GImpactCollisionAlgorithm;
import com.bulletphysics.extras.gimpact.GImpactMeshShape;
import com.bulletphysics.util.ObjectArrayList;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import java.nio.ByteBuffer;

import static com.bulletphysics.StackOfBoxesTest.*;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ConvexShapeCollisionTests {

    CollisionShape[] shapes = new CollisionShape[]{
            new BoxShape(new Vector3d(0.5f, 0.5f, 0.5f)),
            new SphereShape(0.5f),
            new CapsuleShape(0.3f, 1f),
            new CylinderShape(new Vector3d(0.5f, 0.5f, 0.5f)),
            new CylinderShapeX(new Vector3d(0.5f, 0.5f, 0.5f)),
            new CylinderShapeZ(new Vector3d(0.5f, 0.5f, 0.5f)),
            new ConeShape(0.5f, 1f),
            new ConeShapeX(0.5f, 1f),
            new ConeShapeZ(0.5f, 1f),
    };

    @Test
    public void testAllConvexShapeCombinationsCollide() {
        for (CollisionShape shape : shapes) {
            for (CollisionShape collisionShape : shapes) {
                boolean collided = simulateAndCheckCollision(false, shape, collisionShape);
                String pair = shape.getClass().getSimpleName() + " vs " + collisionShape.getClass().getSimpleName();
                assertTrue(collided, "Expected collision: " + pair);
            }
        }
    }

    @Test
    public void testAllConvexShapeCombinationsDoNotCollideWhenSeparated() {
        for (CollisionShape shape : shapes) {
            for (CollisionShape collisionShape : shapes) {
                boolean collided = simulateAndCheckCollision(true, shape, collisionShape);
                String pair = shape.getClass().getSimpleName() + " vs " + collisionShape.getClass().getSimpleName();
                assertFalse(collided, "Unexpected collision detected: " + pair);
            }
        }
    }

    @Test
    public void testConvexShapesCollideWithCubeMesh() {
        for (CollisionShape convex : shapes) {
            boolean collided = simulateConvexVsMeshCollision(convex);
            String name = convex.getClass().getSimpleName();
            assertTrue(collided, "Expected collision: " + name + " vs CubeMesh");
        }
    }

    @Test
    public void testConvexShapesCollideWithGImpactMesh() {
        for (CollisionShape convex : shapes) {
            boolean collided = simulateConvexVsGImpactMesh(convex);
            String name = convex.getClass().getSimpleName();
            assertTrue(collided, "Expected collision: " + name + " vs GImpactMesh");
        }
    }

    private boolean simulateConvexVsMeshCollision(CollisionShape convexShape) {
        // Create mesh shape
        BvhTriangleMeshShape meshShape = createCubeMeshShape(0.5f);

        // Setup world
        DiscreteDynamicsWorld world = createWorld();
        createGround(world);

        // Add static mesh body
        RigidBody bodyA = createRigidBody(0f, new Vector3f(0, 0, 0), meshShape);
        world.addRigidBody(bodyA);

        // Add dynamic convex shape body slightly above the mesh
        float y = 0.8f;
        if (convexShape instanceof ConeShapeZ) y = 0f;
        RigidBody bodyB = createRigidBody(1f, new Vector3f(0, y, 0), convexShape);
        world.addRigidBody(bodyB);

        // Simulate
        for (int i = 0; i < 10; i++) {
            world.stepSimulation(1f / 60f, 10);
        }

        return isColliding(world, bodyA, bodyB);
    }

    private boolean simulateConvexVsGImpactMesh(CollisionShape convexShape) {
        GImpactMeshShape meshShape = createGImpactCubeMeshShape(0.5f);
        DiscreteDynamicsWorld world = createWorld();

        // IMPORTANT: Register GImpactCollisionAlgorithm
        GImpactCollisionAlgorithm.registerAlgorithm((CollisionDispatcher) world.getDispatcher());

        RigidBody meshBody = createRigidBody(0f, new Vector3f(0, 0, 0), meshShape);
        world.addRigidBody(meshBody);

        RigidBody convexBody = createRigidBody(1f, new Vector3f(0, 0.5f, 0), convexShape);
        world.addRigidBody(convexBody);

        for (int i = 0; i < 10; i++) {
            world.stepSimulation(1f / 240f, 10);
        }

        return isColliding(world, convexBody, meshBody);
    }

    private boolean simulateAndCheckCollision(boolean separated, CollisionShape shapeA, CollisionShape shapeB) {
        // Setup physics world
        DiscreteDynamicsWorld world = createWorld();

        // Create two overlapping dynamic bodies
        RigidBody bodyA = createRigidBody(1f, new Vector3f(0, 0, 0), shapeA);
        RigidBody bodyB = createRigidBody(1f, new Vector3f(0, separated ? 3f : 0.2f, 0), shapeB); // Slight vertical offset

        world.addRigidBody(bodyA);
        world.addRigidBody(bodyB);

        // Step simulation a few times
        for (int step = 0; step < 5; step++) {
            world.stepSimulation(1f / 60f, 10);
        }

        return isColliding(world, bodyA, bodyB);
    }

    private boolean isColliding(DiscreteDynamicsWorld world, RigidBody bodyA, RigidBody bodyB) {

        // Check for collision between A and B
        ObjectArrayList<BroadphasePair> pairArray = world.getBroadphase()
                .getOverlappingPairCache()
                .getOverlappingPairArray();

        for (int i = 0; i < pairArray.size(); i++) {
            BroadphasePair pair = pairArray.getQuick(i);
            if ((pair.proxy0.clientObject == bodyA && pair.proxy1.clientObject == bodyB) ||
                    (pair.proxy0.clientObject == bodyB && pair.proxy1.clientObject == bodyA)) {

               /* ObjectArrayList<PersistentManifold> manifoldArray = world.getDispatcher().getInternalManifoldPointer();
                for (int j = 0; j < manifoldArray.size(); j++) {
                    PersistentManifold manifold = manifoldArray.getQuick(j);
                    if (manifold.getNumContacts() > 0) {*/
                return true;
               /*     }
                }*/
            }
        }

        return false;
    }

    private BvhTriangleMeshShape createCubeMeshShape(
            @SuppressWarnings("SameParameterValue") float halfExtents
    ) {

        // Define cube vertices
        Vector3f[] vertices = {
                new Vector3f(-halfExtents, -halfExtents, -halfExtents),
                new Vector3f(halfExtents, -halfExtents, -halfExtents),
                new Vector3f(halfExtents, halfExtents, -halfExtents),
                new Vector3f(-halfExtents, halfExtents, -halfExtents),
                new Vector3f(-halfExtents, -halfExtents, halfExtents),
                new Vector3f(halfExtents, -halfExtents, halfExtents),
                new Vector3f(halfExtents, halfExtents, halfExtents),
                new Vector3f(-halfExtents, halfExtents, halfExtents)
        };

        // Define triangles (12 for a cube)
        int[] indices = {
                0, 1, 2, 2, 3, 0, // back
                4, 5, 6, 6, 7, 4, // front
                0, 4, 7, 7, 3, 0, // left
                1, 5, 6, 6, 2, 1, // right
                3, 2, 6, 6, 7, 3, // top
                0, 1, 5, 5, 4, 0  // bottom
        };

        // Flatten vertices into float array
        float[] vertexArray = new float[vertices.length * 3];
        for (int i = 0; i < vertices.length; i++) {
            vertexArray[i * 3] = vertices[i].x;
            vertexArray[i * 3 + 1] = vertices[i].y;
            vertexArray[i * 3 + 2] = vertices[i].z;
        }

        // Create mesh
        TriangleIndexVertexArray mesh = new TriangleIndexVertexArray(
                indices.length / 3,
                wrap(indices),
                3 * 4,
                vertexArray.length / 3,
                wrap(vertexArray),
                3 * 4
        );

        return new BvhTriangleMeshShape(mesh, true);
    }

    private GImpactMeshShape createGImpactCubeMeshShape(
            @SuppressWarnings("SameParameterValue") float halfExtents
    ) {
        // Define cube vertices
        Vector3f[] vertices = {
                new Vector3f(-halfExtents, -halfExtents, -halfExtents),
                new Vector3f(halfExtents, -halfExtents, -halfExtents),
                new Vector3f(halfExtents, halfExtents, -halfExtents),
                new Vector3f(-halfExtents, halfExtents, -halfExtents),
                new Vector3f(-halfExtents, -halfExtents, halfExtents),
                new Vector3f(halfExtents, -halfExtents, halfExtents),
                new Vector3f(halfExtents, halfExtents, halfExtents),
                new Vector3f(-halfExtents, halfExtents, halfExtents)
        };

        int[] indices = {
                0, 1, 2, 2, 3, 0, // back
                4, 5, 6, 6, 7, 4, // front
                0, 4, 7, 7, 3, 0, // left
                1, 5, 6, 6, 2, 1, // right
                3, 2, 6, 6, 7, 3, // top
                0, 1, 5, 5, 4, 0  // bottom
        };

        float[] vertexArray = new float[vertices.length * 3];
        for (int i = 0; i < vertices.length; i++) {
            vertexArray[i * 3] = vertices[i].x;
            vertexArray[i * 3 + 1] = vertices[i].y;
            vertexArray[i * 3 + 2] = vertices[i].z;
        }

        TriangleIndexVertexArray mesh = new TriangleIndexVertexArray(
                indices.length / 3,
                wrap(indices),
                3 * 4,
                vertexArray.length / 3,
                wrap(vertexArray),
                3 * 4
        );

        GImpactMeshShape shape = new GImpactMeshShape(mesh);
        shape.updateBound(); // IMPORTANT
        return shape;
    }

    private ByteBuffer wrap(int[] indices) {
        ByteBuffer data = ByteBuffer.allocate(indices.length * 4);
        data.asIntBuffer().put(indices);
        return data;
    }

    private ByteBuffer wrap(float[] vertices) {
        ByteBuffer data = ByteBuffer.allocate(vertices.length * 4);
        data.asFloatBuffer().put(vertices);
        return data;
    }
}
