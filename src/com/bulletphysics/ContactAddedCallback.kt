package com.bulletphysics;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;

/**
 * Called when contact has been created between two collision objects. At least
 * one of object must have {@link CollisionFlags#CUSTOM_MATERIAL_CALLBACK} flag set.
 * 
 * @see BulletGlobals#setContactAddedCallback
 * @author jezek2
 */
public interface ContactAddedCallback {

	@SuppressWarnings("UnusedReturnValue")
	boolean contactAdded(ManifoldPoint cp, CollisionObject colObj0, int partId0, int index0, CollisionObject colObj1, int partId1, int index1);
	
}
