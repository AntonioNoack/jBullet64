package com.bulletphysics.collision.shapes;

/**
 * ConeShape implements a cone shape, around the X axis.
 * 
 * @author jezek2
 */
public class ConeShapeX extends ConeShape {

	public ConeShapeX(double radius, double height) {
		super(radius, height);
		setConeUpIndex(0);
	}

}
