package com.bulletphysics.collision.shapes

import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * Cylinder shape around the Z axis.
 *
 * @author jezek2
 */
class CylinderShapeZ(halfExtents: Vector3d) : CylinderShape(halfExtents) {
    
    init {
        upAxis = 2
        recalculateLocalAabb()
    }

    override fun localGetSupportingVertexWithoutMargin(dir: Vector3d, out: Vector3d): Vector3d {
        val halfExtends = getHalfExtentsWithoutMargin(Stack.newVec())
        val result = cylinderLocalSupportZ(halfExtends, dir, out)
        Stack.subVec(1)
        return result
    }

    override fun batchedUnitVectorGetSupportingVertexWithoutMargin(
        dirs: Array<Vector3d>,
        outs: Array<Vector3d>,
        numVectors: Int
    ) {
        val halfExtends = getHalfExtentsWithoutMargin(Stack.newVec())
        for (i in 0 until numVectors) {
            cylinderLocalSupportZ(halfExtends, dirs[i], outs[i])
        }
        Stack.subVec(1)
    }
}
