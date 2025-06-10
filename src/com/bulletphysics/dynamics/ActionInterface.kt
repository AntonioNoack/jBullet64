package com.bulletphysics.dynamics;

import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.linearmath.IDebugDraw;

/**
 * Basic interface to allow actions such as vehicles and characters to be
 * updated inside a {@link DynamicsWorld}.
 *
 * @author tomrbryn
 */
public interface ActionInterface {
    void updateAction(CollisionWorld collisionWorld, double deltaTimeStep);

    void debugDraw(IDebugDraw debugDrawer);
}
