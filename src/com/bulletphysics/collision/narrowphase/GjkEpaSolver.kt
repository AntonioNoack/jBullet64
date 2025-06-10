package com.bulletphysics.collision.narrowphase

import com.bulletphysics.BulletGlobals
import com.bulletphysics.collision.shapes.ConvexShape
import com.bulletphysics.linearmath.MatrixUtil
import com.bulletphysics.linearmath.QuaternionUtil.setRotation
import com.bulletphysics.linearmath.Transform
import com.bulletphysics.linearmath.VectorUtil
import com.bulletphysics.util.ArrayPool
import com.bulletphysics.util.ObjectStackList
import cz.advel.stack.Stack
import java.util.*
import javax.vecmath.Matrix3d
import javax.vecmath.Vector3d
import kotlin.math.abs
import kotlin.math.max

/**
 * GjkEpaSolver contributed under zlib by Nathanael Presson, Nov. 2006.
 *
 * @author jezek2
 */
class GjkEpaSolver {
    val floatArrays: ArrayPool<DoubleArray?> =
        ArrayPool.Companion.get(Double::class.javaPrimitiveType!!)

    val stackMkv = ObjectStackList<VertexAndRay>(VertexAndRay::class.java)
    val stackHe = ObjectStackList<Vec3Node>(Vec3Node::class.java)
    val stackFace = ObjectStackList<Face>(Face::class.java)

    fun pushStack() {
        stackMkv.push()
        stackHe.push()
        stackFace.push()
    }

    fun popStack() {
        stackMkv.pop()
        stackHe.pop()
        stackFace.pop()
    }

    enum class ResultsStatus {
        Separated,  /* Shapes don't penetrate												*/
        Penetrating,  /* Shapes are penetrating												*/
        GJK_Failed,  /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
        EPA_Failed,  /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
    }

    class Results {
        var status: ResultsStatus? = null
        val witnesses /*[2]*/ = arrayOf(Vector3d(), Vector3d())
        val normal: Vector3d = Vector3d()
        var depth: Double = 0.0
        var epaIterations: Int = 0
        var gjkIterations: Int = 0
    }

    /**///////////////////////////////////////////////////////////////////////// */
    class VertexAndRay {
        val w: Vector3d = Vector3d() // Minkowski vertex
        val r: Vector3d = Vector3d() // Ray

        fun set(m: VertexAndRay) {
            w.set(m.w)
            r.set(m.r)
        }
    }

    class Vec3Node {
        val v: Vector3d = Vector3d()
        var n: Vec3Node? = null
    }

    inner class GJK {
        //public btStackAlloc sa;
        //public Block sablock;
        val table: Array<Vec3Node?> = arrayOfNulls<Vec3Node>(GJKHashSize)
        val wrotations /*[2]*/ = arrayOf(Matrix3d(), Matrix3d())
        val positions /*[2]*/ = arrayOf(Vector3d(), Vector3d())
        val shapes: Array<ConvexShape?> = arrayOfNulls(2)
        val simplex = Array(5) { VertexAndRay() }
        val ray: Vector3d = Vector3d()
        /*unsigned*/var order: Int = 0
        /*unsigned*/var iterations: Int = 0
        var margin: Double = 0.0
        var failed: Boolean = false

        constructor()

        @JvmOverloads
        constructor( /*StackAlloc psa,*/
                     wrot0: Matrix3d, pos0: Vector3d, shape0: ConvexShape?,
                     wrot1: Matrix3d, pos1: Vector3d, shape1: ConvexShape?,
                     pmargin: Double = 0.0
        ) {
            init(wrot0, pos0, shape0, wrot1, pos1, shape1, pmargin)
        }

        fun init( /*StackAlloc psa,*/
                  wrot0: Matrix3d, pos0: Vector3d, shape0: ConvexShape?,
                  wrot1: Matrix3d, pos1: Vector3d, shape1: ConvexShape?,
                  pmargin: Double
        ) {
            pushStack()
            wrotations[0].set(wrot0)
            positions[0].set(pos0)
            shapes[0] = shape0
            wrotations[1].set(wrot1)
            positions[1].set(pos1)
            shapes[1] = shape1
            //sa		=psa;
            //sablock	=sa->beginBlock();
            margin = pmargin
            failed = false
        }

        fun destroy() {
            popStack()
        }

        // vdh: very dummy hash
        /*unsigned*/ fun Hash(v: Vector3d): Int {
            val h = (v.x * 15461).toInt() xor (v.y * 83003).toInt() xor (v.z * 15473).toInt()
            return (h * 169639) and GJKHashMask
        }

        fun LocalSupport(
            d: Vector3d, /*unsigned*/i: Int, out: Vector3d
        ): Vector3d {
            val dir = Stack.newVec()
            MatrixUtil.transposeTransform(dir, d, wrotations[i])

            shapes[i]!!.localGetSupportingVertex(dir, out)
            wrotations[i].transform(out)
            out.add(positions[i])
            Stack.subVec(1)

            return out
        }

        fun Support(d: Vector3d, v: VertexAndRay) {
            v.r.set(d)

            val tmp1 = LocalSupport(d, 0, Stack.newVec())

            val tmp = Stack.newVec()
            tmp.set(d)
            tmp.negate()
            val tmp2 = LocalSupport(tmp, 1, Stack.newVec())

            v.w.sub(tmp1, tmp2)
            v.w.scaleAdd(margin, d, v.w)
            Stack.subVec(3)
        }

        fun FetchSupport(): Boolean {
            val h = Hash(ray)
            var e = table[h]
            while (e != null) {
                if (e.v.equals(ray)) {
                    --order
                    return false
                } else {
                    e = e.n
                }
            }
            //e = (He*)sa->allocate(sizeof(He));
            //e = new He();
            e = stackHe.get()
            e!!.v.set(ray)
            e.n = table[h]
            table[h] = e
            Support(ray, simplex[++order])
            return (ray.dot(simplex[order].w) > 0)
        }

        fun SolveSimplex2(ao: Vector3d, ab: Vector3d): Boolean {
            if (ab.dot(ao) >= 0) {
                val cabo = Stack.borrowVec()
                cabo.cross(ab, ao)
                if (cabo.lengthSquared() > GJKSqInSimplexEpsilon) {
                    ray.cross(cabo, ab)
                } else {
                    return true
                }
            } else {
                order = 0
                simplex[0].set(simplex[1])
                ray.set(ao)
            }
            return false
        }

        fun SolveSimplex3(ao: Vector3d, ab: Vector3d, ac: Vector3d): Boolean {
            val tmp = Stack.newVec()
            tmp.cross(ab, ac)
            val result = SolveSimplex3a(ao, ab, ac, tmp)
            Stack.subVec(1)
            return result
        }

        fun SolveSimplex3a(ao: Vector3d, ab: Vector3d, ac: Vector3d, cabc: Vector3d): Boolean {
            // TODO: optimize

            val tmp = Stack.newVec()
            tmp.cross(cabc, ab)

            val tmp2 = Stack.newVec()
            tmp2.cross(cabc, ac)

            val result: Boolean
            if (tmp.dot(ao) < -GJKInSimplexEpsilon) {
                order = 1
                simplex[0].set(simplex[1])
                simplex[1].set(simplex[2])
                result = SolveSimplex2(ao, ab)
            } else if (tmp2.dot(ao) > +GJKInSimplexEpsilon) {
                order = 1
                simplex[1].set(simplex[2])
                result = SolveSimplex2(ao, ac)
            } else {
                val d = cabc.dot(ao)
                if (abs(d) > GJKInSimplexEpsilon) {
                    if (d > 0) {
                        ray.set(cabc)
                    } else {
                        ray.negate(cabc)

                        val swapTmp = VertexAndRay()
                        swapTmp.set(simplex[0])
                        simplex[0].set(simplex[1])
                        simplex[1]!!.set(swapTmp)
                    }
                    result = false
                } else {
                    result = true
                }
            }
            Stack.subVec(2)
            return result
        }

        fun SolveSimplex4(ao: Vector3d, ab: Vector3d, ac: Vector3d, ad: Vector3d): Boolean {
            // TODO: optimize

            val crs = Stack.newVec()

            val tmp = Stack.newVec()
            tmp.cross(ab, ac)

            val tmp2 = Stack.newVec()
            tmp2.cross(ac, ad)

            val tmp3 = Stack.newVec()
            tmp3.cross(ad, ab)

            val result: Boolean
            if (tmp.dot(ao) > GJKInSimplexEpsilon) {
                crs.set(tmp)
                order = 2
                simplex[0].set(simplex[1])
                simplex[1].set(simplex[2])
                simplex[2].set(simplex[3])
                result = SolveSimplex3a(ao, ab, ac, crs)
            } else if (tmp2.dot(ao) > GJKInSimplexEpsilon) {
                crs.set(tmp2)
                order = 2
                simplex[2].set(simplex[3])
                result = SolveSimplex3a(ao, ac, ad, crs)
            } else if (tmp3.dot(ao) > GJKInSimplexEpsilon) {
                crs.set(tmp3)
                order = 2
                simplex[1].set(simplex[0])
                simplex[0].set(simplex[2])
                simplex[2].set(simplex[3])
                result = SolveSimplex3a(ao, ad, ab, crs)
            } else result = true
            Stack.subVec(4)
            return result
        }

        fun SearchOrigin(): Boolean {
            val tmp = Stack.newVec()
            tmp.set(1.0, 0.0, 0.0)
            val result = SearchOrigin(tmp)
            Stack.subVec(1)
            return result
        }

        fun SearchOrigin(initray: Vector3d): Boolean {
            val tmp1 = Stack.newVec()
            val tmp2 = Stack.newVec()
            val tmp3 = Stack.newVec()
            val tmp4 = Stack.newVec()

            iterations = 0
            order = -1
            failed = false
            ray.set(initray)
            ray.normalize()

            Arrays.fill(table, null)

            FetchSupport()
            ray.negate(simplex[0].w)
            while (iterations < GJKMaxIterations) {
                val rl = ray.length()
                ray.scale(1.0 / (if (rl > 0.0) rl else 1.0))
                if (FetchSupport()) {
                    var found = false
                    when (order) {
                        1 -> {
                            tmp1.negate(simplex[1].w)
                            tmp2.sub(simplex[0].w, simplex[1].w)
                            found = SolveSimplex2(tmp1, tmp2)
                        }
                        2 -> {
                            tmp1.negate(simplex[2].w)
                            tmp2.sub(simplex[1].w, simplex[2].w)
                            tmp3.sub(simplex[0].w, simplex[2].w)
                            found = SolveSimplex3(tmp1, tmp2, tmp3)
                        }
                        3 -> {
                            tmp1.negate(simplex[3].w)
                            tmp2.sub(simplex[2].w, simplex[3].w)
                            tmp3.sub(simplex[1].w, simplex[3].w)
                            tmp4.sub(simplex[0].w, simplex[3].w)
                            found = SolveSimplex4(tmp1, tmp2, tmp3, tmp4)
                        }
                    }
                    if (found) {
                        Stack.subVec(4)
                        return true
                    }
                } else {
                    Stack.subVec(4)
                    return false
                }
                ++iterations
            }
            failed = true
            Stack.subVec(4)
            return false
        }

        fun EncloseOrigin(): Boolean {
            val tmp = Stack.newVec()
            val tmp1 = Stack.newVec()
            val tmp2 = Stack.newVec()

            when (order) {
                0 -> {}
                1 -> {
                    val ab = Stack.newVec()
                    ab.sub(simplex[1].w, simplex[0].w)

                    val b = arrayOf<Vector3d>(Stack.newVec(), Stack.newVec(), Stack.newVec())
                    b[0].set(1.0, 0.0, 0.0)
                    b[1].set(0.0, 1.0, 0.0)
                    b[2].set(0.0, 0.0, 1.0)

                    b[0].cross(ab, b[0])
                    b[1].cross(ab, b[1])
                    b[2].cross(ab, b[2])

                    val m = doubleArrayOf(b[0].lengthSquared(), b[1].lengthSquared(), b[2].lengthSquared())

                    val tmpQuat = Stack.newQuat()
                    tmp.normalize(ab)
                    setRotation(tmpQuat, tmp, cst2Pi / 3f)

                    val r = Stack.newMat()
                    MatrixUtil.setRotation(r, tmpQuat)

                    val w = Stack.newVec()
                    w.set(b[if (m[0] > m[1]) if (m[0] > m[2]) 0 else 2 else if (m[1] > m[2]) 1 else 2])

                    tmp.normalize(w)
                    Support(tmp, simplex[4])
                    r.transform(w)
                    tmp.normalize(w)
                    Support(tmp, simplex[2])
                    r.transform(w)
                    tmp.normalize(w)
                    Support(tmp, simplex[3])
                    r.transform(w)
                    order = 4

                    Stack.subVec(8)
                    Stack.subMat(1)
                    Stack.subQuat(1)

                    return true
                }
                2 -> {
                    tmp1.sub(simplex[1].w, simplex[0].w)
                    tmp2.sub(simplex[2].w, simplex[0].w)
                    val n = Stack.newVec()
                    n.cross(tmp1, tmp2)
                    n.normalize()

                    Support(n, simplex[3])

                    tmp.negate(n)
                    Support(tmp, simplex[4])
                    order = 4

                    Stack.subVec(4)
                    return true
                }
                3, 4 -> {
                    Stack.subVec(3)
                    return true
                }
            }
            Stack.subVec(3)
            return false
        }
    }

    class Face {
        val vertices: Array<VertexAndRay?> = arrayOfNulls(3)
        val children: Array<Face?> = arrayOfNulls<Face>(3)
        val e: IntArray = IntArray(3)
        val n: Vector3d = Vector3d()
        var d: Double = 0.0
        var mark: Int = 0
        var prev: Face? = null
        var next: Face? = null
    }

    inner class EPA(pgjk: GJK) {
        var gjk: GJK = pgjk

        //public btStackAlloc* sa;
        var root: Face? = null
        var nfaces: Int = 0
        var iterations: Int = 0
        val features: Array<Array<Vector3d>> = Array(2) { Array(3) { Vector3d() } }
        val nearest /*[2]*/ = arrayOf(Vector3d(), Vector3d())
        val normal: Vector3d = Vector3d()
        var depth: Double = 0.0
        var failed: Boolean = false

        fun getCoordinates(face: Face, out: Vector3d): Vector3d {
            val tmp = Stack.newVec()
            val tmp1 = Stack.newVec()
            val tmp2 = Stack.newVec()

            val o = Stack.newVec()
            o.scale(-face.d, face.n)

            val a = floatArrays.getFixed(3)

            tmp1.sub(face.vertices[0]!!.w, o)
            tmp2.sub(face.vertices[1]!!.w, o)
            tmp.cross(tmp1, tmp2)
            a!![0] = tmp.length()

            tmp1.sub(face.vertices[1]!!.w, o)
            tmp2.sub(face.vertices[2]!!.w, o)
            tmp.cross(tmp1, tmp2)
            a[1] = tmp.length()

            tmp1.sub(face.vertices[2]!!.w, o)
            tmp2.sub(face.vertices[0]!!.w, o)
            tmp.cross(tmp1, tmp2)
            a[2] = tmp.length()

            val sm = a[0] + a[1] + a[2]

            out.set(a[1], a[2], a[0])
            out.scale(1.0 / (if (sm > 0.0) sm else 1.0))

            floatArrays.release(a)
            Stack.subVec(4)

            return out
        }

        fun findBestFace(): Face? {
            var bf: Face? = null
            if (root != null) {
                var cf = root
                var bd: Double = cstInf
                do {
                    if (cf!!.d < bd) {
                        bd = cf.d
                        bf = cf
                    }
                } while (null != (cf.next.also { cf = it }))
            }
            return bf
        }

        fun Set(f: Face, a: VertexAndRay, b: VertexAndRay, c: VertexAndRay): Boolean {
            val tmp1 = Stack.newVec()
            val tmp2 = Stack.newVec()
            val tmp3 = Stack.newVec()

            val nrm = Stack.newVec()
            tmp1.sub(b.w, a.w)
            tmp2.sub(c.w, a.w)
            nrm.cross(tmp1, tmp2)

            val len = nrm.length()

            tmp1.cross(a.w, b.w)
            tmp2.cross(b.w, c.w)
            tmp3.cross(c.w, a.w)

            val valid = (tmp1.dot(nrm) >= -EPAInFaceEpsilon) &&
                    (tmp2.dot(nrm) >= -EPAInFaceEpsilon) &&
                    (tmp3.dot(nrm) >= -EPAInFaceEpsilon)

            f.vertices[0] = a
            f.vertices[1] = b
            f.vertices[2] = c
            f.mark = 0
            f.n.scale(1.0 / (if (len > 0.0) len else cstInf), nrm)
            f.d = max(0.0, -f.n.dot(a.w))
            Stack.subVec(4)
            return valid
        }

        fun newFace(a: VertexAndRay, b: VertexAndRay, c: VertexAndRay): Face {
            //Face pf = new Face();
            val pf = stackFace.get()
            if (Set(pf, a, b, c)) {
                if (root != null) {
                    root!!.prev = pf
                }
                pf.prev = null
                pf.next = root
                root = pf
                ++nfaces
            } else {
                pf.next = null
                pf.prev = pf.next
            }
            return (pf)
        }

        fun detach(face: Face) {
            if (face.prev != null || face.next != null) {
                --nfaces
                if (face === root) {
                    root = face.next
                    root!!.prev = null
                } else {
                    if (face.next == null) {
                        face.prev!!.next = null
                    } else {
                        checkNotNull(face.prev)
                        face.prev!!.next = face.next
                        face.next!!.prev = face.prev
                    }
                }
                face.next = null
                face.prev = face.next
            }
        }

        fun link(f0: Face, e0: Int, f1: Face, e1: Int) {
            f0.children[e0] = f1
            f1.e[e1] = e0
            f1.children[e1] = f0
            f0.e[e0] = e1
        }

        fun support(w: Vector3d): VertexAndRay {
            //Mkv v = new Mkv();
            val v = stackMkv.get()
            gjk.Support(w, v)
            return v
        }

        fun buildHorizon(markId: Int, w: VertexAndRay, f: Face, e: Int, cf: Array<Face?>, ff: Array<Face?>): Int {
            var ne = 0
            if (f.mark != markId) {
                val e1: Int = mod3[e + 1]
                if ((f.n.dot(w.w) + f.d) > 0) {
                    val nf = newFace(f.vertices[e1]!!, f.vertices[e]!!, w)
                    link(nf, 0, f, e)
                    if (cf[0] != null) {
                        link(cf[0]!!, 1, nf, 2)
                    } else {
                        ff[0] = nf
                    }
                    cf[0] = nf
                    ne = 1
                } else {
                    val e2: Int = mod3[e + 2]
                    detach(f)
                    f.mark = markId
                    ne += buildHorizon(markId, w, f.children[e1]!!, f.e[e1], cf, ff)
                    ne += buildHorizon(markId, w, f.children[e2]!!, f.e[e2], cf, ff)
                }
            }
            return (ne)
        }

        @JvmOverloads
        fun evaluatePD(accuracy: Double = EPAAccuracy): Double {
            pushStack()
            val tmp = Stack.newVec()
            try {
                var bestface: Face? = null
                var markid = 1
                depth = -cstInf
                normal.set(0.0, 0.0, 0.0)
                root = null
                nfaces = 0
                iterations = 0
                failed = false
                /* Prepare hull */
                if (gjk.EncloseOrigin()) {
                    var pfidx_ptr: Array<IntArray>? = null
                    var pfidx_index = 0

                    var nfidx = 0
                    var peidx_ptr: Array<IntArray>? = null
                    var peidx_index = 0

                    var neidx = 0
                    val basemkv = arrayOfNulls<VertexAndRay>(5)
                    val baseFaces = arrayOfNulls<Face>(6)
                    when (gjk.order) {
                        3 -> {
                            pfidx_ptr = tetrahedron_fidx
                            nfidx = 4
                            peidx_ptr = tetrahedron_eidx
                            neidx = 6
                        }
                        4 -> {
                            pfidx_ptr = hexahedron_fidx
                            nfidx = 6
                            peidx_ptr = hexahedron_eidx
                            neidx = 9
                        }
                    }
                    for (i in 0..gjk.order) {
                        basemkv[i] = VertexAndRay()
                        basemkv[i]!!.set(gjk.simplex[i])
                    }
                    for (i in 0 until nfidx) {
                        baseFaces[i] = newFace(
                            basemkv[pfidx_ptr!![pfidx_index][0]]!!,
                            basemkv[pfidx_ptr[pfidx_index][1]]!!,
                            basemkv[pfidx_ptr[pfidx_index][2]]!!
                        )
                        pfidx_index++
                    }
                    repeat(neidx) {
                        link(
                            baseFaces[peidx_ptr!![peidx_index][0]]!!,
                            peidx_ptr[peidx_index][1],
                            baseFaces[peidx_ptr[peidx_index][2]]!!,
                            peidx_ptr[peidx_index][3]
                        )
                        peidx_index++
                    }
                }
                if (0 == nfaces) {
                    return depth
                }
                /* Expand hull		*/
                while (iterations < EPAMaxIterations) {
                    val bf = findBestFace()
                    if (bf != null) {
                        tmp.negate(bf.n)
                        val w = support(tmp)
                        val d = bf.n.dot(w.w) + bf.d
                        bestface = bf
                        if (d < -accuracy) {
                            val cf = arrayOf<Face?>(null)
                            val ff = arrayOf<Face?>(null)
                            var nf = 0
                            detach(bf)
                            bf.mark = ++markid
                            for (i in 0..2) {
                                nf += buildHorizon(markid, w, bf.children[i]!!, bf.e[i], cf, ff)
                            }
                            if (nf <= 2) {
                                break
                            }
                            link(cf[0]!!, 1, ff[0]!!, 2)
                        } else {
                            break
                        }
                    } else {
                        break
                    }
                    ++iterations
                }
                /* Extract contact	*/
                if (bestface != null) {
                    val b = getCoordinates(bestface, Stack.newVec())
                    normal.set(bestface.n)
                    depth = max(0.0, bestface.d)
                    for (i in 0..1) {
                        val s = if (i != 0) -1.0 else 1.0
                        for (j in 0..2) {
                            tmp.scale(s, bestface.vertices[j]!!.r)
                            gjk.LocalSupport(tmp, i, features[i][j])
                        }
                    }

                    val tmp1 = Stack.newVec()
                    val tmp2 = Stack.newVec()
                    val tmp3 = Stack.newVec()

                    tmp1.scale(b.x, features[0][0])
                    tmp2.scale(b.y, features[0][1])
                    tmp3.scale(b.z, features[0][2])
                    VectorUtil.add(nearest[0], tmp1, tmp2, tmp3)

                    tmp1.scale(b.x, features[1][0])
                    tmp2.scale(b.y, features[1][1])
                    tmp3.scale(b.z, features[1][2])
                    VectorUtil.add(nearest[1], tmp1, tmp2, tmp3)
                    Stack.subVec(4)
                } else {
                    failed = true
                }
                return depth
            } finally {
                popStack()
                Stack.subVec(1)
            }
        }
    }

    /**///////////////////////////////////////////////////////////////////////// */
    private val gjk = GJK()

    fun collide(
        shape0: ConvexShape?, wtrs0: Transform,
        shape1: ConvexShape?, wtrs1: Transform,
        radialMargin: Double,  /*,
			btStackAlloc* stackAlloc*/
        results: Results
    ): Boolean {
        // Initialize

        results.witnesses[0].set(0.0, 0.0, 0.0)
        results.witnesses[1].set(0.0, 0.0, 0.0)
        results.normal.set(0.0, 0.0, 0.0)
        results.depth = 0.0
        results.status = ResultsStatus.Separated
        results.epaIterations = 0
        results.gjkIterations = 0
        /* Use GJK to locate origin		*/
        gjk.init( /*stackAlloc,*/
            wtrs0.basis, wtrs0.origin, shape0,
            wtrs1.basis, wtrs1.origin, shape1,
            radialMargin + EPAAccuracy
        )
        try {
            val collide = gjk.SearchOrigin()
            results.gjkIterations = gjk.iterations + 1
            if (collide) {
                /* Then EPA for penetration depth	*/
                val epa = EPA(gjk)
                val pd = epa.evaluatePD()
                results.epaIterations = epa.iterations + 1
                if (pd > 0) {
                    results.status = ResultsStatus.Penetrating
                    results.normal.set(epa.normal)
                    results.depth = pd
                    results.witnesses[0].set(epa.nearest[0])
                    results.witnesses[1].set(epa.nearest[1])
                    return true
                } else {
                    if (epa.failed) {
                        results.status = ResultsStatus.EPA_Failed
                    }
                }
            } else {
                if (gjk.failed) {
                    results.status = ResultsStatus.GJK_Failed
                }
            }
            return false
        } finally {
            gjk.destroy()
        }
    }

    companion object {
        /**///////////////////////////////////////////////////////////////////////// */
        private val cstInf = BulletGlobals.SIMD_INFINITY
        private val cst2Pi = BulletGlobals.SIMD_2_PI
        private const val GJKMaxIterations = 128
        private val GJKHashSize = 1 shl 6
        private val GJKHashMask: Int = GJKHashSize - 1
        private const val GJKInSimplexEpsilon = 0.0001
        private val GJKSqInSimplexEpsilon: Double = GJKInSimplexEpsilon * GJKInSimplexEpsilon
        private const val EPAMaxIterations = 256
        private const val EPAInFaceEpsilon = 0.01
        private const val EPAAccuracy = 0.001

        /**///////////////////////////////////////////////////////////////////////// */
        private val mod3 = intArrayOf(0, 1, 2, 0, 1)

        private val tetrahedron_fidx /*[4][3]*/ =
            arrayOf<IntArray>(intArrayOf(2, 1, 0), intArrayOf(3, 0, 1), intArrayOf(3, 1, 2), intArrayOf(3, 2, 0))
        private val tetrahedron_eidx /*[6][4]*/ = arrayOf<IntArray>(
            intArrayOf(0, 0, 2, 1),
            intArrayOf(0, 1, 1, 1),
            intArrayOf(0, 2, 3, 1),
            intArrayOf(1, 0, 3, 2),
            intArrayOf(2, 0, 1, 2),
            intArrayOf(3, 0, 2, 2)
        )

        private val hexahedron_fidx /*[6][3]*/ = arrayOf<IntArray>(
            intArrayOf(2, 0, 4),
            intArrayOf(4, 1, 2),
            intArrayOf(1, 4, 0),
            intArrayOf(0, 3, 1),
            intArrayOf(0, 2, 3),
            intArrayOf(1, 3, 2)
        )
        private val hexahedron_eidx /*[9][4]*/ = arrayOf<IntArray>(
            intArrayOf(0, 0, 4, 0),
            intArrayOf(0, 1, 2, 1),
            intArrayOf(0, 2, 1, 2),
            intArrayOf(1, 1, 5, 2),
            intArrayOf(1, 0, 2, 0),
            intArrayOf(2, 2, 3, 2),
            intArrayOf(3, 1, 5, 0),
            intArrayOf(3, 0, 4, 2),
            intArrayOf(5, 1, 4, 1)
        )
    }
}
