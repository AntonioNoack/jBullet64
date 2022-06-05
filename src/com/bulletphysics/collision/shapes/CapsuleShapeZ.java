package com.bulletphysics.collision.shapes;

/**
 * CapsuleShapeZ represents a capsule around the Z axis.<p>
 * 
 * The total height is <code>height+2*radius</code>, so the height is just the
 * height between the center of each "sphere" of the capsule caps.
 * 
 * @author jezek2
 */
public class CapsuleShapeZ extends CapsuleShape {

	public CapsuleShapeZ(double radius, double height) {
		upAxis = 2;
		implicitShapeDimensions.set(radius, radius, 0.5 * height);
	}
	
	@Override
	public String getName() {
		return "CapsuleZ";
	}

}
