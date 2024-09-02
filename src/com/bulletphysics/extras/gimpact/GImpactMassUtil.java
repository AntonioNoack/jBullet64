package com.bulletphysics.extras.gimpact;

import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
class GImpactMassUtil {

	public static Vector3d get_point_inertia(Vector3d point, double mass, Vector3d out) {
		double x2 = point.x * point.x;
		double y2 = point.y * point.y;
		double z2 = point.z * point.z;
		out.set(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
		return out;
	}
	
}
