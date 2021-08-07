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

/**
 * Current state of contact solver.
 * 
 * @author jezek2
 */
public class ContactSolverInfo {

	public double tau = 0.6;
	public double damping = 1;
	public double friction = 0.3;
	public double timeStep;
	public double restitution = 0;
	public int numIterations = 10;
	public double maxErrorReduction = 20;
	public double sor = 1.3;
	public double erp = 0.2; // used as Baumgarte factor
	public double erp2 = 0.1; // used in Split Impulse
	public boolean splitImpulse = false;
	public double splitImpulsePenetrationThreshold = -0.02;
	public double linearSlop = 0.0;
	public double warmstartingFactor = 0.85;
	
	public int solverMode = SolverMode.SOLVER_RANDMIZE_ORDER | SolverMode.SOLVER_CACHE_FRIENDLY | SolverMode.SOLVER_USE_WARMSTARTING;

	public ContactSolverInfo() {
	}
	
	public ContactSolverInfo(ContactSolverInfo g) {
		tau = g.tau;
		damping = g.damping;
		friction = g.friction;
		timeStep = g.timeStep;
		restitution = g.restitution;
		numIterations = g.numIterations;
		maxErrorReduction = g.maxErrorReduction;
		sor = g.sor;
		erp = g.erp;
	}
	
}
