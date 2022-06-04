package com.bulletphysics.linearmath;

/**
 * MotionState allows the dynamics world to synchronize the updated world transforms
 * with graphics. For optimizations, potentially only moving objects get synchronized
 * (using {@link #setWorldTransform setWorldTransform} method).
 * 
 * @author jezek2
 */
public abstract class MotionState {

	/**
	 * Returns world transform.
	 */
	public abstract Transform getWorldTransform(Transform out);

	/**
	 * Sets world transform. This method is called by JBullet whenever an active
	 * object represented by this MotionState is moved or rotated.
	 */
	public abstract void setWorldTransform(Transform worldTrans);
	
}
