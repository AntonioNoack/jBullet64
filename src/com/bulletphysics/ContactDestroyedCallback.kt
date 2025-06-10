package com.bulletphysics

/**
 * Called when contact has been destroyed between two collision objects.
 *
 * @see BulletGlobals.setContactDestroyedCallback
 *
 * @author jezek2
 */
interface ContactDestroyedCallback {
    fun contactDestroyed(userPersistentData: Any): Boolean
}
