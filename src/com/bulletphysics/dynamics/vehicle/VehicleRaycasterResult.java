package com.bulletphysics.dynamics.vehicle;

import javax.vecmath.Vector3d;

/**
 * Vehicle raycaster result.
 * 
 * @author jezek2
 */
public class VehicleRaycasterResult {
	
	public final Vector3d hitPointInWorld  = new Vector3d();
	public final Vector3d hitNormalInWorld  = new Vector3d();
	public double distFraction = -1.0;

}
