package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.dynamics.RigidBody;

/**
 * Contact solving function.
 * 
 * @author jezek2
 */
public interface ContactSolverFunc {

	double resolveContact(RigidBody body1, RigidBody body2, ManifoldPoint contactPoint, ContactSolverInfo info);
	
}
