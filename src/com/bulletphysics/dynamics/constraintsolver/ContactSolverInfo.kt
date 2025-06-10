package com.bulletphysics.dynamics.constraintsolver;

/**
 * Current state of contact solver.
 * 
 * @author jezek2
 */
public class ContactSolverInfo {

	public double tau = 0.6;
	public double damping = 1.0;
	public double friction = 0.3;
	public double timeStep;
	public double restitution = 0.0;
	public int numIterations = 10;
	public double maxErrorReduction = 20.0;

	/**
	 * Successive over-relaxation
	 * */
	public double sor = 1.3;

	/**
	 * used as Baumgarte factor
	 * */
	public double erp = 0.2;

	/**
	 * used in Split Impulse
	 * */
	public double erp2 = 0.1;
	public boolean splitImpulse = false;
	public double splitImpulsePenetrationThreshold = -0.02f;
	public double linearSlop = 0.0;
	public double warmstartingFactor = 0.85f;
	
	public int solverMode = SolverMode.SOLVER_RANDOMIZE_ORDER | SolverMode.SOLVER_CACHE_FRIENDLY | SolverMode.SOLVER_USE_WARMSTARTING;

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
		erp2 = g.erp2;
		splitImpulse = g.splitImpulse;
		splitImpulsePenetrationThreshold = g.splitImpulsePenetrationThreshold;
		linearSlop = g.linearSlop;
		warmstartingFactor = g.warmstartingFactor;
		solverMode = g.solverMode;
	}
	
}
