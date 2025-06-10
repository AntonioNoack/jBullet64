package com.bulletphysics.collision.broadphase

import com.bulletphysics.linearmath.VectorUtil.setMax
import com.bulletphysics.linearmath.VectorUtil.setMin
import cz.advel.stack.Stack
import javax.vecmath.Vector3d
import kotlin.math.abs

/**
 * Dbvt implementation by Nathanael Presson
 * @author jezek2
 */
class DbvtAabbMm {
    private val mi = Vector3d()
    private val mx = Vector3d()

    fun set(o: DbvtAabbMm) {
        mi.set(o.mi)
        mx.set(o.mx)
    }

    fun Center(out: Vector3d): Vector3d {
        out.add(mi, mx)
        out.scale(0.5)
        return out
    }

    fun Mins(): Vector3d {
        return mi
    }

    fun Maxs(): Vector3d {
        return mx
    }

    fun Expand(e: Vector3d) {
        mi.sub(e)
        mx.add(e)
    }

    fun SignedExpand(e: Vector3d) {
        if (e.x > 0) {
            mx.x += e.x
        } else {
            mi.x += e.x
        }

        if (e.y > 0) {
            mx.y += e.y
        } else {
            mi.y += e.y
        }

        if (e.z > 0) {
            mx.z += e.z
        } else {
            mi.z += e.z
        }
    }

    fun Contain(a: DbvtAabbMm): Boolean {
        return ((mi.x <= a.mi.x) &&
                (mi.y <= a.mi.y) &&
                (mi.z <= a.mi.z) &&
                (mx.x >= a.mx.x) &&
                (mx.y >= a.mx.y) &&
                (mx.z >= a.mx.z))
    }

    companion object {
        fun swap(p1: DbvtAabbMm, p2: DbvtAabbMm) {
            val tmp = Stack.borrowVec()

            tmp.set(p1.mi)
            p1.mi.set(p2.mi)
            p2.mi.set(tmp)

            tmp.set(p1.mx)
            p1.mx.set(p2.mx)
            p2.mx.set(tmp)
        }

        fun FromCE(c: Vector3d, e: Vector3d, out: DbvtAabbMm): DbvtAabbMm {
            out.mi.sub(c, e)
            out.mx.add(c, e)
            return out
        }

        fun FromCR(c: Vector3d, r: Double, out: DbvtAabbMm): DbvtAabbMm {
            val tmp = Stack.newVec()
            tmp.set(r, r, r)
            return FromCE(c, tmp, out)
        }

        fun FromMM(mi: Vector3d, mx: Vector3d, out: DbvtAabbMm): DbvtAabbMm {
            out.mi.set(mi)
            out.mx.set(mx)
            return out
        }

        fun Intersect(a: DbvtAabbMm, b: DbvtAabbMm): Boolean {
            return ((a.mi.x <= b.mx.x) &&
                    (a.mx.x >= b.mi.x) &&
                    (a.mi.y <= b.mx.y) &&
                    (a.mx.y >= b.mi.y) &&
                    (a.mi.z <= b.mx.z) &&
                    (a.mx.z >= b.mi.z))
        }

        fun Proximity(a: DbvtAabbMm, b: DbvtAabbMm): Double {
            val ai = a.mi
            val ax = a.mx
            val bi = b.mi
            val bx = b.mx
            return abs((ai.x + ax.x) - (bi.x + bx.x)) + abs((ai.y + ax.y) - (bi.y + bx.y)) + abs((ai.z + ax.z) - (bi.z + bx.z))
        }

        fun Merge(a: DbvtAabbMm, b: DbvtAabbMm, r: DbvtAabbMm) {
            setMin(r.mi, a.mi, b.mi)
            setMax(r.mx, a.mx, b.mx)
        }

        fun NotEqual(a: DbvtAabbMm, b: DbvtAabbMm): Boolean {
            return ((a.mi.x != b.mi.x) ||
                    (a.mi.y != b.mi.y) ||
                    (a.mi.z != b.mi.z) ||
                    (a.mx.x != b.mx.x) ||
                    (a.mx.y != b.mx.y) ||
                    (a.mx.z != b.mx.z))
        }
    }
}
