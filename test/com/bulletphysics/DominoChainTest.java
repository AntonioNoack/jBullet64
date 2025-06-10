package com.bulletphysics;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import org.junit.jupiter.api.Test;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import static com.bulletphysics.StackOfBoxesTest.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class DominoChainTest {

    @Test
    public void testDominoTopple() {
        runDominoToppleTest(3);
        runDominoToppleTest(20);
        runDominoToppleTest(100);
    }

    private void runDominoToppleTest(int dominoCount) {
        DiscreteDynamicsWorld world = createWorld();
        createGround(world);

        // Domino dimensions (in meters)
        float scaleForStability = 2f;
        float height = 0.05f * scaleForStability;
        float width = 0.025f * scaleForStability;
        float depth = 0.01f * scaleForStability;
        CollisionShape dominoShape = new BoxShape(new Vector3d(width / 2, height / 2, depth / 2));

        // Positioning
        RigidBody[] dominos = new RigidBody[dominoCount];
        float spacing = depth + 0.002f; // slight gap
        float startX = 0f;
        float y = height * 0.85f;
        float z = 0f;

        for (int i = 0; i < dominoCount; i++) {
            dominos[i] = createRigidBody(0.05f, new Vector3f(startX + i * spacing, y, z), dominoShape);
            dominos[i].friction = 0.5f;
            dominos[i].restitution = 0.1f;
            world.addRigidBody(dominos[i]);
        }

        // Push first domino with angular impulse to tip it forward
        RigidBody firstDomino = dominos[0];
        // firstDomino.applyTorqueImpulse(new Vector3d(0, 0, -0.02f)); // tip around X-axis (forward)
        firstDomino.applyImpulse(new Vector3d(0.02f, 0, 0), new Vector3d(0, height / 2, depth / 2));

        int lastFallenCount = 0;
        float timeStep = 1f / 240f;
        int numSteps = 2000;
        for (int i = 0; i < numSteps; i++) {
            world.stepSimulation(timeStep, 10);

            int fallen = 0;
            for (int d = 0; d < dominoCount; d++) {
                Transform tf = new Transform();
                dominos[d].getMotionState().getWorldTransform(tf);
                Vector3d up = new Vector3d();
                tf.basis.getColumn(1, up); // local Y axis
                double dot = up.dot(new Vector3d(0, 1, 0)); // how aligned with world up
                if (dot < 0.7f) { // ~45 degrees tipped
                    fallen++;
                }
            }

            if (fallen != lastFallenCount) {
                System.out.printf("[" + dominoCount + "] Step %d: %d fallen%n", i, fallen);
                lastFallenCount = fallen;
            }
        }

        // Check that the last domino has fallen (tipped significantly)
        Transform lastTransform = new Transform();
        dominos[dominoCount - 1].getMotionState().getWorldTransform(lastTransform);

        Vector3d upVector = new Vector3d();
        lastTransform.basis.getColumn(1, upVector); // Y-axis of the last domino

        double verticalDot = upVector.dot(new Vector3d(0, 1, 0)); // close to 1 if upright
        boolean isFallen = verticalDot < 0.7f; // less than ~45° from upright

        assertTrue(isFallen, "Domino chain failed for N=" + dominoCount + ". Final up vector dot: " + verticalDot);
    }
}