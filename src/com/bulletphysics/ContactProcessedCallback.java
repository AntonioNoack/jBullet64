package com.bulletphysics;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;

/**
 * Called when existing contact between two collision objects has been processed.
 * 
 * @see BulletGlobals#setContactProcessedCallback
 * @author jezek2
 */
public interface ContactProcessedCallback {
	
	@SuppressWarnings("UnusedReturnValue")
    boolean contactProcessed(ManifoldPoint cp, Object body0, Object body1);

}
