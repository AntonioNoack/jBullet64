package com.bulletphysics.collision.shapes

/**
 * ConeShape implements a cone shape, around the X axis.
 *
 * @author jezek2
 */
class ConeShapeX(radius: Double, height: Double) : ConeShape(radius, height) {
    init {
        coneUpIndex = 0
    }
}
