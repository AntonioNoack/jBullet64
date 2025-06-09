package com.bulletphysics;

/**
 * Called when contact has been destroyed between two collision objects.
 *
 * @see BulletGlobals#setContactDestroyedCallback
 * @author jezek2
 */
public interface ContactDestroyedCallback {

	@SuppressWarnings("UnusedReturnValue")
	boolean contactDestroyed(Object userPersistentData);
	
}
