package com.bulletphysics.collision.broadphase;

import com.bulletphysics.linearmath.IDebugDraw;

/**
 * Current state of {@link Dispatcher}.
 *
 * @author jezek2
 */
public class DispatcherInfo {

    public double timeStep;
    public int stepCount;
    public DispatchFunc dispatchFunc;
    public double timeOfImpact;
    public IDebugDraw debugDraw;
    public double allowedCcdPenetration = 0.04f;

    public DispatcherInfo() {
        dispatchFunc = DispatchFunc.DISPATCH_DISCRETE;
        timeOfImpact = 1.0;
    }

}
