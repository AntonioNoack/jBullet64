package com.bulletphysics.collision.shapes

import com.bulletphysics.linearmath.MiscUtil
import com.bulletphysics.linearmath.convexhull.HullDesc
import com.bulletphysics.linearmath.convexhull.HullFlags
import com.bulletphysics.linearmath.convexhull.HullLibrary
import com.bulletphysics.linearmath.convexhull.HullResult
import com.bulletphysics.util.IntArrayList
import com.bulletphysics.util.ObjectArrayList
import cz.advel.stack.Stack
import javax.vecmath.Vector3d

/**
 * ShapeHull takes a [ConvexShape], builds the convex hull using [HullLibrary]
 * and provides triangle indices and vertices.
 *
 * @author jezek2
 */
class ShapeHull(var shape: ConvexShape) {

    var numIndices: Int
    val vertexPointer: ObjectArrayList<Vector3d> = ObjectArrayList<Vector3d>()
    val indexPointer: IntArrayList = IntArrayList()
    val unitSpherePoints: ObjectArrayList<Vector3d> = ObjectArrayList<Vector3d>()

    fun buildHull(margin: Double): Boolean {
        val norm = Stack.newVec()

        var numSampleDirections: Int = NUM_UNIT_SPHERE_POINTS
        run {
            val numPDA = shape.numPreferredPenetrationDirections
            if (numPDA != 0) {
                for (i in 0 until numPDA) {
                    shape.getPreferredPenetrationDirection(i, norm)
                    unitSpherePoints.getQuick(numSampleDirections)!!.set(norm)
                    numSampleDirections++
                }
            }
        }

        val supportPoints = ObjectArrayList<Vector3d>()
        MiscUtil.resize(
            supportPoints,
            NUM_UNIT_SPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2,
            Vector3d::class.java
        )

        for (i in 0 until numSampleDirections) {
            shape.localGetSupportingVertex(unitSpherePoints.getQuick(i)!!, supportPoints.getQuick(i)!!)
        }

        val hd = HullDesc()
        hd.flags = HullFlags.TRIANGLES
        hd.vcount = numSampleDirections

        //#ifdef BT_USE_DOUBLE_PRECISION
        //hd.mVertices = &supportPoints[0];
        //hd.mVertexStride = sizeof(btVector3);
        //#else
        hd.vertices = supportPoints

        //hd.vertexStride = 3 * 4;
        //#endif
        val hl = HullLibrary()
        val hr = HullResult()
        if (!hl.createConvexHull(hd, hr)) {
            return false
        }

        MiscUtil.resize(this.vertexPointer, hr.numOutputVertices, Vector3d::class.java)

        for (i in 0 until hr.numOutputVertices) {
            vertexPointer.getQuick(i)!!.set(hr.outputVertices.getQuick(i))
        }
        numIndices = hr.numIndices
        MiscUtil.resize(this.indexPointer, numIndices, 0)
        for (i in 0 until numIndices) {
            indexPointer.set(i, hr.indices.get(i))
        }

        // free temporary hull result that we just copied
        hl.releaseResult(hr)

        return true
    }

    fun numTriangles(): Int {
        return numIndices / 3
    }

    fun numVertices(): Int {
        return vertexPointer.size
    }

    fun numIndices(): Int {
        return numIndices
    }

    init {
        this.vertexPointer.clear()
        this.indexPointer.clear()
        this.numIndices = 0

        MiscUtil.resize(
            unitSpherePoints,
            NUM_UNIT_SPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2,
            Vector3d::class.java
        )
        for (i in constUnitSpherePoints.indices) {
            unitSpherePoints.getQuick(i)!!.set(constUnitSpherePoints.getQuick(i))
        }
    }

    companion object {
        /** ///////////////////////////////////////////////////////////////////////// */
        private const val NUM_UNIT_SPHERE_POINTS = 42

        private val constUnitSpherePoints = ObjectArrayList<Vector3d?>()

        init {
            val pts: ObjectArrayList<Vector3d?> = constUnitSpherePoints
            pts.add(Vector3d(+0.000000, -0.000000, -1.000000))
            pts.add(Vector3d(+0.723608, -0.525725, -0.447219))
            pts.add(Vector3d(-0.276388, -0.850649, -0.447219))
            pts.add(Vector3d(-0.894426, -0.000000, -0.447216))
            pts.add(Vector3d(-0.276388, +0.850649, -0.447220))
            pts.add(Vector3d(+0.723608, +0.525725, -0.447219))
            pts.add(Vector3d(+0.276388, -0.850649, +0.447220))
            pts.add(Vector3d(-0.723608, -0.525725, +0.447219))
            pts.add(Vector3d(-0.723608, +0.525725, +0.447219))
            pts.add(Vector3d(+0.276388, +0.850649, +0.447219))
            pts.add(Vector3d(+0.894426, +0.000000, +0.447216))
            pts.add(Vector3d(-0.000000, +0.000000, +1.000000))
            pts.add(Vector3d(+0.425323, -0.309011, -0.850654))
            pts.add(Vector3d(-0.162456, -0.499995, -0.850654))
            pts.add(Vector3d(+0.262869, -0.809012, -0.525738))
            pts.add(Vector3d(+0.425323, +0.309011, -0.850654))
            pts.add(Vector3d(+0.850648, -0.000000, -0.525736))
            pts.add(Vector3d(-0.525730, -0.000000, -0.850652))
            pts.add(Vector3d(-0.688190, -0.499997, -0.525736))
            pts.add(Vector3d(-0.162456, +0.499995, -0.850654))
            pts.add(Vector3d(-0.688190, +0.499997, -0.525736))
            pts.add(Vector3d(+0.262869, +0.809012, -0.525738))
            pts.add(Vector3d(+0.951058, +0.309013, +0.000000))
            pts.add(Vector3d(+0.951058, -0.309013, +0.000000))
            pts.add(Vector3d(+0.587786, -0.809017, +0.000000))
            pts.add(Vector3d(+0.000000, -1.000000, +0.000000))
            pts.add(Vector3d(-0.587786, -0.809017, +0.000000))
            pts.add(Vector3d(-0.951058, -0.309013, -0.000000))
            pts.add(Vector3d(-0.951058, +0.309013, -0.000000))
            pts.add(Vector3d(-0.587786, +0.809017, -0.000000))
            pts.add(Vector3d(-0.000000, +1.000000, -0.000000))
            pts.add(Vector3d(+0.587786, +0.809017, -0.000000))
            pts.add(Vector3d(+0.688190, -0.499997, +0.525736))
            pts.add(Vector3d(-0.262869, -0.809012, +0.525738))
            pts.add(Vector3d(-0.850648, +0.000000, +0.525736))
            pts.add(Vector3d(-0.262869, +0.809012, +0.525738))
            pts.add(Vector3d(+0.688190, +0.499997, +0.525736))
            pts.add(Vector3d(+0.525730, +0.000000, +0.850652))
            pts.add(Vector3d(+0.162456, -0.499995, +0.850654))
            pts.add(Vector3d(-0.425323, -0.309011, +0.850654))
            pts.add(Vector3d(-0.425323, +0.309011, +0.850654))
            pts.add(Vector3d(+0.162456, +0.499995, +0.850654))
        }
    }
}
