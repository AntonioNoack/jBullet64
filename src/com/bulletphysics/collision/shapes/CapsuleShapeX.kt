package com.bulletphysics.collision.shapes

/**
 * CapsuleShapeX represents a capsule around the X axis.
 *
 * The total height is `height+2*radius`, so the height is just the
 * height between the center of each "sphere" of the capsule caps.
 *
 * @author jezek2
 */
class CapsuleShapeX(radius: Double, height: Double) : CapsuleShape(radius, height, 0)
