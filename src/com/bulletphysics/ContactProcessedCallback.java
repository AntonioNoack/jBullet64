package com.bulletphysics;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;

/**
 * Called when existing contact between two collision objects has been processed.
 * 
 * @see BulletGlobals#setContactProcessedCallback
 * @author jezek2
 */
public interface ContactProcessedCallback {
	
	@SuppressWarnings("UnusedReturnValue")
    boolean contactProcessed(ManifoldPoint cp, CollisionObject body0, CollisionObject body1);

}
