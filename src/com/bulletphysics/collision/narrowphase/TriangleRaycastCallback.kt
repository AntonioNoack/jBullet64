package com.bulletphysics.collision.narrowphase

import com.bulletphysics.collision.shapes.TriangleCallback
import com.bulletphysics.linearmath.VectorUtil.setInterpolate3
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * @author jezek2
 */
abstract class TriangleRaycastCallback(from: Vector3d, to: Vector3d) : TriangleCallback {

    val from: Vector3d = Vector3d()
    val to: Vector3d = Vector3d()

    var hitFraction: Double

    init {
        this.from.set(from)
        this.to.set(to)
        this.hitFraction = 1.0
    }

    override fun processTriangle(triangle: Array<Vector3d>, partId: Int, triangleIndex: Int) {
        val vert0 = triangle[0]
        val vert1 = triangle[1]
        val vert2 = triangle[2]

        val v10 = Stack.newVec()
        v10.sub(vert1, vert0)

        val v20 = Stack.newVec()
        v20.sub(vert2, vert0)

        val triangleNormal = Stack.newVec()
        triangleNormal.cross(v10, v20)

        val dist = vert0.dot(triangleNormal)
        var dist_a = triangleNormal.dot(from)
        dist_a -= dist
        var dist_b = triangleNormal.dot(to)
        dist_b -= dist

        if (dist_a * dist_b >= 0.0) {
            return  // same sign
        }

        val proj_length = dist_a - dist_b
        val distance = (dist_a) / (proj_length)

        // Now we have the intersection point on the plane, we'll see if it's inside the triangle
        // Add an epsilon as a tolerance for the raycast,
        // in case the ray hits exacly on the edge of the triangle.
        // It must be scaled for the triangle size.
        if (distance < hitFraction) {
            var edge_tolerance = triangleNormal.lengthSquared()
            edge_tolerance *= -0.0001
            val point = Vector3d()
            setInterpolate3(point, from, to, distance)
            run {
                val v0p = Stack.newVec()
                v0p.sub(vert0, point)
                val v1p = Stack.newVec()
                v1p.sub(vert1, point)
                val cp0 = Stack.newVec()
                cp0.cross(v0p, v1p)

                if (cp0.dot(triangleNormal) >= edge_tolerance) {
                    val v2p = Stack.newVec()
                    v2p.sub(vert2, point)
                    val cp1 = Stack.newVec()
                    cp1.cross(v1p, v2p)

                    if (cp1.dot(triangleNormal) >= edge_tolerance) {
                        cp1.cross(v2p, v0p)
                        if (cp1.dot(triangleNormal) >= edge_tolerance) {
                            if (dist_a > 0.0) {
                                hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex)
                            } else {
                                val tmp = Stack.newVec()
                                tmp.negate(triangleNormal)
                                hitFraction = reportHit(tmp, distance, partId, triangleIndex)
                                Stack.subVec(1)
                            }
                        }
                    }
                    Stack.subVec(2)
                }
                Stack.subVec(3)
            }
        }

        Stack.subVec(3)
    }

    abstract fun reportHit(hitNormalLocal: Vector3d, hitFraction: Double, partId: Int, triangleIndex: Int): Double
}
