package com.bulletphysics.dynamics.vehicle;

import javax.vecmath.Vector3d;

/**
 * VehicleRaycaster is provides interface for between vehicle simulation and raycasting.
 * 
 * @author jezek2
 */
public abstract class VehicleRaycaster {

	public abstract Object castRay(Vector3d from, Vector3d to, VehicleRaycasterResult result);
	
}
