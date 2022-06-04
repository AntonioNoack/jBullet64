package com.bulletphysics.linearmath;

/**
 * DefaultMotionState provides a common implementation to synchronize world transforms
 * with offsets.
 * 
 * @author jezek2
 */
public class DefaultMotionState extends MotionState {

	/** Current interpolated world transform, used to draw object. */
	public final Transform graphicsWorldTrans = new Transform();
	
	/** Center of mass offset transform, used to adjust graphics world transform. */
	public final Transform centerOfMassOffset = new Transform();
	
	/** Initial world transform. */
	public final Transform startWorldTrans = new Transform();
	
	/**
	 * Creates a new DefaultMotionState with all transforms set to identity.
	 */
	public DefaultMotionState() {
		graphicsWorldTrans.setIdentity();
		centerOfMassOffset.setIdentity();
		startWorldTrans.setIdentity();
	}

	/**
	 * Creates a new DefaultMotionState with initial world transform and center
	 * of mass offset transform set to identity.
	 */
	public DefaultMotionState(Transform startTrans) {
		this.graphicsWorldTrans.set(startTrans);
		centerOfMassOffset.setIdentity();
		this.startWorldTrans.set(startTrans);
	}
	
	/**
	 * Creates a new DefaultMotionState with initial world transform and center
	 * of mass offset transform.
	 */
	public DefaultMotionState(Transform startTrans, Transform centerOfMassOffset) {
		this.graphicsWorldTrans.set(startTrans);
		this.centerOfMassOffset.set(centerOfMassOffset);
		this.startWorldTrans.set(startTrans);
	}
	
	public Transform getWorldTransform(Transform out) {
		out.inverse(centerOfMassOffset);
		out.mul(graphicsWorldTrans);
		return out;
	}

	public void setWorldTransform(Transform centerOfMassWorldTrans) {
		graphicsWorldTrans.set(centerOfMassWorldTrans);
		graphicsWorldTrans.mul(centerOfMassOffset);
	}

}
