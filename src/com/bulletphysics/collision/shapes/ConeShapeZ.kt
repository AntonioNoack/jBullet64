package com.bulletphysics.collision.shapes

/**
 * ConeShape implements a cone shape, around the Z axis.
 *
 * @author jezek2
 */
class ConeShapeZ(radius: Double, height: Double) : ConeShape(radius, height) {
    init {
        coneUpIndex = 2
    }
}
