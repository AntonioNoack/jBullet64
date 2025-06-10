package com.bulletphysics.dynamics;

/**
 * Callback called for each internal tick.
 *
 * @author jezek2
 * @see DynamicsWorld#setInternalTickCallback
 */
public interface InternalTickCallback {
    void internalTick(DynamicsWorld world, double timeStep);
}
