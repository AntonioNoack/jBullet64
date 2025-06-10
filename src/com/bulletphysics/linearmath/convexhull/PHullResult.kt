package com.bulletphysics.linearmath.convexhull

import com.bulletphysics.util.IntArrayList
import com.bulletphysics.util.ObjectArrayList
import javax.vecmath.Vector3d

/**
 *
 * @author jezek2
 */
internal class PHullResult {
    var vertexCount: Int = 0
    var indexCount: Int = 0
    var faceCount: Int = 0
    var vertices: ObjectArrayList<Vector3d>? = null
    var indices: IntArrayList = IntArrayList()
}
