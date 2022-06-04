package com.bulletphysics;

/**
 * Called when contact has been destroyed between two collision objects.
 *
 * @see BulletGlobals#setContactDestroyedCallback
 * @author jezek2
 */
public abstract class ContactDestroyedCallback {

	@SuppressWarnings("UnusedReturnValue")
	public abstract boolean contactDestroyed(Object userPersistentData);
	
}
