package com.bulletphysics.dynamics;

/**
 * Callback called for each internal tick.
 * 
 * @see DynamicsWorld#setInternalTickCallback
 * @author jezek2
 */
public interface InternalTickCallback {

	void internalTick(DynamicsWorld world, double timeStep);
	
}
