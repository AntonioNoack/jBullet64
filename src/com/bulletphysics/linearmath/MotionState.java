package com.bulletphysics.linearmath;

/**
 * MotionState allows the dynamics world to synchronize the updated world transforms
 * with graphics. For optimizations, potentially only moving objects get synchronized
 * (using {@link #setWorldTransform setWorldTransform} method).
 * 
 * @author jezek2
 */
public interface MotionState {

	/**
	 * Returns world transform.
	 */
	Transform getWorldTransform(Transform out);

	/**
	 * Sets world transform. This method is called by JBullet whenever an active
	 * object represented by this MotionState is moved or rotated.
	 */
	void setWorldTransform(Transform worldTrans);
	
}
