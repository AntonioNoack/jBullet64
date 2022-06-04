package com.bulletphysics.dynamics.constraintsolver;

/**
 * Current state of contact solver.
 * 
 * @author jezek2
 */
public class ContactSolverInfo {

	public static final int SOLVER_RANDOMIZE_ORDER = 1;
	public static final int SOLVER_FRICTION_SEPARATE = 2;
	public static final int SOLVER_USE_WARM_STARTING = 4;
	public static final int SOLVER_CACHE_FRIENDLY = 8;

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
	
	public int solverMode = SOLVER_RANDOMIZE_ORDER | SOLVER_CACHE_FRIENDLY | SOLVER_USE_WARM_STARTING;

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
