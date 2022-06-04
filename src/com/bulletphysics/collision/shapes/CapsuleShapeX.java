package com.bulletphysics.collision.shapes;

/**
 * CapsuleShapeX represents a capsule around the X axis.<p>
 * 
 * The total height is <code>height+2*radius</code>, so the height is just the
 * height between the center of each "sphere" of the capsule caps.
 * 
 * @author jezek2
 */
public class CapsuleShapeX extends CapsuleShape {

	public CapsuleShapeX(double radius, double height) {
		upAxis = 0;
		implicitShapeDimensions.set(0.5f * height, radius, radius);
	}
	
	@Override
	public String getName() {
		return "CapsuleX";
	}

}
