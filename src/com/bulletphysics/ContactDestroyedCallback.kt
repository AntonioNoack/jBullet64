package com.bulletphysics

/**
 * Called when contact has been destroyed between two collision objects.
 *
 * @see BulletGlobals.setContactDestroyedCallback
 *
 * @author jezek2
 */
fun interface ContactDestroyedCallback {
    fun contactDestroyed(userPersistentData: Any): Boolean
}
