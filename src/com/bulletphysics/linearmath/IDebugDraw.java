package com.bulletphysics.linearmath;

import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * IDebugDraw interface class allows hooking up a debug renderer to visually debug
 * simulations.<p>
 * 
 * Typical use case: create a debug drawer object, and assign it to a {@link CollisionWorld}
 * or {@link DynamicsWorld} using setDebugDrawer and call debugDrawWorld.<p>
 * 
 * A class that implements the IDebugDraw interface has to implement the drawLine
 * method at a minimum.
 * 
 * @author jezek2
 */
public abstract class IDebugDraw {

	public abstract void drawLine(Vector3d from, Vector3d to, Vector3d color);
	
	public void drawTriangle(Vector3d v0, Vector3d v1, Vector3d v2, Vector3d n0, Vector3d n1, Vector3d n2, Vector3d color, double alpha) {
		drawTriangle(v0, v1, v2, color, alpha);
	}
	
	public void drawTriangle(Vector3d v0, Vector3d v1, Vector3d v2, Vector3d color, double alpha) {
		drawLine(v0, v1, color);
		drawLine(v1, v2, color);
		drawLine(v2, v0, color);
	}

	public abstract void drawContactPoint(Vector3d PointOnB, Vector3d normalOnB, double distance, int lifeTime, Vector3d color);

	public abstract void reportErrorWarning(String warningString);

	public abstract void draw3dText(Vector3d location, String textString);

	public abstract void setDebugMode(int debugMode);

	public abstract int getDebugMode();

	public void drawAabb(Vector3d from, Vector3d to, Vector3d color) {
		Vector3d halfExtents = Stack.newVec(to);
		halfExtents.sub(from);
		halfExtents.scale(0.5);

		Vector3d center = Stack.newVec(to);
		center.add(from);
		center.scale(0.5);

		int i, j;

		Vector3d edgecoord = Stack.newVec();
		edgecoord.set(1.0, 1.0, 1.0);
		Vector3d pa = Stack.newVec(), pb = Stack.newVec();
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 3; j++) {
				pa.set(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y, edgecoord.z * halfExtents.z);
				pa.add(center);

				int othercoord = j % 3;

				VectorUtil.mulCoord(edgecoord, othercoord, -1.0);
				pb.set(edgecoord.x * halfExtents.x, edgecoord.y * halfExtents.y, edgecoord.z * halfExtents.z);
				pb.add(center);

				drawLine(pa, pb, color);
			}
			edgecoord.set(-1.0, -1.0, -1.0);
			if (i < 3) {
				VectorUtil.mulCoord(edgecoord, i, -1.0);
			}
		}
	}
}
