package com.bulletphysics;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;

/**
 * Called when existing contact between two collision objects has been processed.
 * 
 * @see BulletGlobals#setContactProcessedCallback
 * @author jezek2
 */
public abstract class ContactProcessedCallback {

	@SuppressWarnings("UnusedReturnValue")
	public abstract boolean contactProcessed(ManifoldPoint cp, Object body0, Object body1);

}
