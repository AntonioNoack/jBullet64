/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.dynamics.constraintsolver;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * Stores some extra information to each contact point. It is not in the contact
 * point, because that want to keep the collision detection independent from the
 * constraint solver.
 * 
 * @author jezek2
 */
public class ConstraintPersistentData {
	
	/** total applied impulse during most recent frame */
	public double appliedImpulse = 0.0;
	public double prevAppliedImpulse = 0.0;
	public double accumulatedTangentImpulse0 = 0.0;
	public double accumulatedTangentImpulse1 = 0.0;

	public double jacDiagABInv = 0.0;
	public double jacDiagABInvTangent0;
	public double jacDiagABInvTangent1;
	public int persistentLifeTime = 0;
	public double restitution = 0.0;
	public double friction = 0.0;
	public double penetration = 0.0;
	public final Vector3d frictionWorldTangential0 = new Vector3d();
	public final Vector3d frictionWorldTangential1 = new Vector3d();

	public final Vector3d frictionAngularComponent0A = new Vector3d();
	public final Vector3d frictionAngularComponent0B = new Vector3d();
	public final Vector3d frictionAngularComponent1A = new Vector3d();
	public final Vector3d frictionAngularComponent1B = new Vector3d();

	//some data doesn't need to be persistent over frames: todo: clean/reuse this
	public final Vector3d angularComponentA = new Vector3d();
	public final Vector3d angularComponentB = new Vector3d();

	public ContactSolverFunc contactSolverFunc = null;
	public ContactSolverFunc frictionSolverFunc = null;
	
	public void reset() {
		appliedImpulse = 0.0;
		prevAppliedImpulse = 0.0;
		accumulatedTangentImpulse0 = 0.0;
		accumulatedTangentImpulse1 = 0.0;

		jacDiagABInv = 0.0;
		jacDiagABInvTangent0 = 0.0;
		jacDiagABInvTangent1 = 0.0;
		persistentLifeTime = 0;
		restitution = 0.0;
		friction = 0.0;
		penetration = 0.0;
		frictionWorldTangential0.set(0.0, 0.0, 0.0);
		frictionWorldTangential1.set(0.0, 0.0, 0.0);

		frictionAngularComponent0A.set(0.0, 0.0, 0.0);
		frictionAngularComponent0B.set(0.0, 0.0, 0.0);
		frictionAngularComponent1A.set(0.0, 0.0, 0.0);
		frictionAngularComponent1B.set(0.0, 0.0, 0.0);

		angularComponentA.set(0.0, 0.0, 0.0);
		angularComponentB.set(0.0, 0.0, 0.0);

		contactSolverFunc = null;
		frictionSolverFunc = null;
	}
	
}
