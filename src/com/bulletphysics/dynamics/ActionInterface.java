package com.bulletphysics.dynamics;

import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.linearmath.IDebugDraw;

/**
 * Basic interface to allow actions such as vehicles and characters to be
 * updated inside a {@link DynamicsWorld}.
 *
 * @author tomrbryn
 */
public abstract class ActionInterface {

	public abstract void updateAction(CollisionWorld collisionWorld, double deltaTimeStep);

	public abstract void debugDraw(IDebugDraw debugDrawer);

}
