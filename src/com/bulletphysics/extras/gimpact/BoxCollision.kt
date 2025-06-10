package com.bulletphysics.extras.gimpact

import com.bulletphysics.BulletGlobals
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.VectorUtil.getCoord
import com.bulletphysics.linearmath.VectorUtil.setCoord
import cz.advel.stack.Stack
import javax.vecmath.Matrix3d
import javax.vecmath.Vector3d
import javax.vecmath.Vector4d
import kotlin.math.abs

/**
 * @author jezek2
 */
internal object BoxCollision {

    const val BOX_PLANE_EPSILON: Double = 0.000001

    fun absGreater(i: Double, j: Double): Boolean {
        return abs(i) > j
    }

    fun max(a: Double, b: Double, c: Double): Double {
        return kotlin.math.max(a, kotlin.math.max(b, c))
    }

    fun min(a: Double, b: Double, c: Double): Double {
        return kotlin.math.min(a, kotlin.math.min(b, c))
    }

    /**
     * Returns the dot product between a vec3 and the col of a matrix.
     */
    fun matXVec(mat: Matrix3d, vec3: Vector3d, column: Int): Double {
        return vec3.x * mat.getElement(0, column) + vec3.y * mat.getElement(1, column) + vec3.z * mat.getElement(
            2,
            column
        )
    }

    /**///////////////////////////////////////////////////////////////////////// */
    class BoxBoxTransformCache {
        val T1to0: Vector3d = Vector3d() // Transforms translation of model1 to model 0
        val R1to0: Matrix3d = Matrix3d() // Transforms Rotation of model1 to model 0, equal  to R0' * R1
        val AR: Matrix3d = Matrix3d() // Absolute value of m_R1to0

        fun calcAbsoluteMatrix() {
            //static const btVector3 vepsi(1e-6f,1e-6f,1e-6f);
            //m_AR[0] = vepsi + m_R1to0[0].absolute();
            //m_AR[1] = vepsi + m_R1to0[1].absolute();
            //m_AR[2] = vepsi + m_R1to0[2].absolute();
            for (i in 0..2) {
                for (j in 0..2) {
                    AR.setElement(i, j, 1e-6 + abs(R1to0.getElement(i, j)))
                }
            }
        }

        /**
         * Calc the transformation relative  1 to 0. Inverts matrices by transposing.
         */
        fun calcFromHomogenic(trans0: Transform, trans1: Transform) {
            val tmpTrans = Stack.newTrans()
            tmpTrans.inverse(trans0)
            tmpTrans.mul(trans1)

            T1to0.set(tmpTrans.origin)
            R1to0.set(tmpTrans.basis)
            Stack.subTrans(1)

            calcAbsoluteMatrix()
        }

        fun transform(point: Vector3d, out: Vector3d): Vector3d {
            var point = point
            val tmp = Stack.newVec()

            if (point === out) {
                point = Stack.borrowVec(point)
            }
            R1to0.getRow(0, tmp)
            out.x = tmp.dot(point) + T1to0.x
            R1to0.getRow(1, tmp)
            out.y = tmp.dot(point) + T1to0.y
            R1to0.getRow(2, tmp)
            out.z = tmp.dot(point) + T1to0.z

            Stack.subVec(1)
            return out
        }
    }

    /**///////////////////////////////////////////////////////////////////////// */
    class AABB {
        @JvmField
        val min: Vector3d = Vector3d()

        @JvmField
        val max: Vector3d = Vector3d()

        constructor()

        constructor(V1: Vector3d, V2: Vector3d, V3: Vector3d) {
            calcFromTriangle(V1, V2, V3)
        }

        constructor(V1: Vector3d, V2: Vector3d, V3: Vector3d, margin: Double) {
            calcFromTriangleMargin(V1, V2, V3, margin)
        }

        constructor(other: AABB) {
            set(other)
        }

        constructor(other: AABB, margin: Double) : this(other) {
            min.x -= margin
            min.y -= margin
            min.z -= margin
            max.x += margin
            max.y += margin
            max.z += margin
        }

        fun init(V1: Vector3d, V2: Vector3d, V3: Vector3d, margin: Double) {
            calcFromTriangleMargin(V1, V2, V3, margin)
        }

        fun set(other: AABB) {
            min.set(other.min)
            max.set(other.max)
        }

        fun invalidate() {
            min.set(BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY, BulletGlobals.SIMD_INFINITY)
            max.set(-BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY, -BulletGlobals.SIMD_INFINITY)
        }

        fun incrementMargin(margin: Double) {
            min.x -= margin
            min.y -= margin
            min.z -= margin
            max.x += margin
            max.y += margin
            max.z += margin
        }

        fun calcFromTriangle(V1: Vector3d, V2: Vector3d, V3: Vector3d) {
            min.x = min(V1.x, V2.x, V3.x)
            min.y = min(V1.y, V2.y, V3.y)
            min.z = min(V1.z, V2.z, V3.z)

            max.x = max(V1.x, V2.x, V3.x)
            max.y = max(V1.y, V2.y, V3.y)
            max.z = max(V1.z, V2.z, V3.z)
        }

        fun calcFromTriangleMargin(V1: Vector3d, V2: Vector3d, V3: Vector3d, margin: Double) {
            calcFromTriangle(V1, V2, V3)
            min.x -= margin
            min.y -= margin
            min.z -= margin
            max.x += margin
            max.y += margin
            max.z += margin
        }

        /**
         * Apply a transform to an AABB.
         */
        fun applyTransform(trans: Transform) {
            val tmp = Stack.newVec()

            val center = Stack.newVec()
            center.add(max, min)
            center.scale(0.5)

            val extends_ = Stack.newVec()
            extends_.sub(max, center)

            // Compute new center
            trans.transform(center)

            val textends = Stack.newVec()

            trans.basis.getRow(0, tmp)
            tmp.absolute()
            textends.x = extends_.dot(tmp)

            trans.basis.getRow(1, tmp)
            tmp.absolute()
            textends.y = extends_.dot(tmp)

            trans.basis.getRow(2, tmp)
            tmp.absolute()
            textends.z = extends_.dot(tmp)

            min.sub(center, textends)
            max.add(center, textends)

            Stack.subVec(4)
        }

        /**
         * Merges a Box.
         */
        fun merge(box: AABB) {
            min.x = kotlin.math.min(min.x, box.min.x)
            min.y = kotlin.math.min(min.y, box.min.y)
            min.z = kotlin.math.min(min.z, box.min.z)

            max.x = kotlin.math.max(max.x, box.max.x)
            max.y = kotlin.math.max(max.y, box.max.y)
            max.z = kotlin.math.max(max.z, box.max.z)
        }

        /**
         * Gets the extend and center.
         */
        fun getCenterExtend(center: Vector3d, extend: Vector3d) {
            center.add(max, min)
            center.scale(0.5)
            extend.sub(max, center)
        }

        fun hasCollision(other: AABB): Boolean {
            return !(min.x > other.max.x) && !(max.x < other.min.x) && !(min.y > other.max.y) && !(max.y < other.min.y) && !(min.z > other.max.z) && !(max.z < other.min.z)
        }

        /**
         * Finds the Ray intersection parameter.
         *
         * @param origin a vec3 with the origin of the ray
         * @param dir    a vec3 with the direction of the ray
         */
        fun collideRay(origin: Vector3d, dir: Vector3d): Boolean {
            val extents = Stack.newVec()
            val center = Stack.newVec()
            getCenterExtend(center, extents)
            Stack.subVec(2)

            val Dx = origin.x - center.x
            if (absGreater(Dx, extents.x) && Dx * dir.x >= 0.0) return false

            val Dy = origin.y - center.y
            if (absGreater(Dy, extents.y) && Dy * dir.y >= 0.0) return false

            val Dz = origin.z - center.z
            if (absGreater(Dz, extents.z) && Dz * dir.z >= 0.0) return false

            var f = dir.y * Dz - dir.z * Dy
            if (abs(f) > extents.y * abs(dir.z) + extents.z * abs(dir.y)) return false

            f = dir.z * Dx - dir.x * Dz
            if (abs(f) > extents.x * abs(dir.z) + extents.z * abs(dir.x)) return false

            f = dir.x * Dy - dir.y * Dx
            if (abs(f) > extents.x * abs(dir.y) + extents.y * abs(dir.x)) return false

            return true
        }

        fun projectionInterval(direction: Vector3d, vmin: DoubleArray, vmax: DoubleArray) {
            val tmp = Stack.newVec()

            val center = Stack.newVec()
            val extend = Stack.newVec()
            getCenterExtend(center, extend)

            val fOrigin = direction.dot(center)
            tmp.absolute(direction)
            val fMaximumExtent = extend.dot(tmp)
            vmin[0] = fOrigin - fMaximumExtent
            vmax[0] = fOrigin + fMaximumExtent
            Stack.subVec(3)
        }

        fun planeClassify(plane: Vector4d): PlaneIntersectionType {
            val tmp = Stack.newVec()

            val min = Stack.newDoublePtr()
            val max = Stack.newDoublePtr()
            tmp.set(plane.x, plane.y, plane.z)
            projectionInterval(tmp, min, max)
            Stack.subVec(1)
            Stack.subDoublePtr(2)

            if (plane.w > max[0] + BOX_PLANE_EPSILON) {
                return PlaneIntersectionType.BACK_PLANE // 0
            }

            if (plane.w + BOX_PLANE_EPSILON >= min[0]) {
                return PlaneIntersectionType.COLLIDE_PLANE //1
            }

            return PlaneIntersectionType.FRONT_PLANE //2
        }

        /**
         * transcache is the transformation cache from box to this AABB.
         */
        fun overlappingTransCache(box: AABB, transcache: BoxBoxTransformCache, fulltest: Boolean): Boolean {
            val tmp = Stack.newVec()

            // Taken from OPCODE
            val ea = Stack.newVec()
            val eb = Stack.newVec() //extends
            val ca = Stack.newVec()
            val cb = Stack.newVec() //extends
            getCenterExtend(ca, ea)
            box.getCenterExtend(cb, eb)

            val T = Stack.newVec()
            var t: Double
            var t2: Double

            try {
                // Class I : A's basis vectors
                for (i in 0..2) {
                    transcache.R1to0.getRow(i, tmp)
                    setCoord(T, i, tmp.dot(cb) + getCoord(transcache.T1to0, i) - getCoord(ca, i))

                    transcache.AR.getRow(i, tmp)
                    t = tmp.dot(eb) + getCoord(ea, i)
                    if (absGreater(getCoord(T, i), t)) {
                        return false
                    }
                }
                // Class II : B's basis vectors
                for (i in 0..2) {
                    t = matXVec(transcache.R1to0, T, i)
                    t2 = matXVec(transcache.AR, ea, i) + getCoord(eb, i)
                    if (absGreater(t, t2)) {
                        return false
                    }
                }
                // Class III : 9 cross products
                if (fulltest) {
                    for (i in 0..2) {
                        val m = (i + 1) % 3
                        val n = (i + 2) % 3
                        val o = if (i == 0) 1 else 0
                        val p = if (i == 2) 1 else 2
                        for (j in 0..2) {
                            val q = if (j == 2) 1 else 2
                            val r = if (j == 0) 1 else 0
                            t = getCoord(T, n) * transcache.R1to0.getElement(m, j) -
                                    getCoord(T, m) * transcache.R1to0.getElement(n, j)
                            t2 = getCoord(ea, o) * transcache.AR.getElement(p, j) +
                                    getCoord(ea, p) * transcache.AR.getElement(o, j) +
                                    getCoord(eb, r) * transcache.AR.getElement(i, q) +
                                    getCoord(eb, q) * transcache.AR.getElement(i, r)
                            if (absGreater(t, t2)) {
                                return false
                            }
                        }
                    }
                }
                return true
            } finally {
                Stack.subVec(6)
            }
        }
    }
}
