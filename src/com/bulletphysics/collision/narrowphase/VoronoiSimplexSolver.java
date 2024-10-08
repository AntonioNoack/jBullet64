package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;
import cz.advel.stack.StaticAlloc;

import javax.vecmath.Vector3d;

/**
 * VoronoiSimplexSolver is an implementation of the closest point distance algorithm
 * from a 1-4 points simplex to the origin. Can be used with GJK, as an alternative
 * to Johnson distance algorithm.
 *
 * @author jezek2
 */
public class VoronoiSimplexSolver extends SimplexSolverInterface {

    //protected final BulletStack stack = BulletStack.get();
    protected final ObjectPool<SubSimplexClosestResult> subsimplexResultsPool = ObjectPool.get(SubSimplexClosestResult.class);

    private static final int VORONOI_SIMPLEX_MAX_VERTICES = 5;

    private static final int VERTEX_A = 0;
    private static final int VERTEX_B = 1;
    private static final int VERTEX_C = 2;

    public int numVertices;

    public final Vector3d[] simplexVectorW = new Vector3d[VORONOI_SIMPLEX_MAX_VERTICES];
    public final Vector3d[] simplexPointsP = new Vector3d[VORONOI_SIMPLEX_MAX_VERTICES];
    public final Vector3d[] simplexPointsQ = new Vector3d[VORONOI_SIMPLEX_MAX_VERTICES];

    public final Vector3d cachedP1 = new Vector3d();
    public final Vector3d cachedP2 = new Vector3d();
    public final Vector3d cachedV = new Vector3d();
    public final Vector3d lastW = new Vector3d();
    public boolean cachedValidClosest;

    public final SubSimplexClosestResult cachedBC = new SubSimplexClosestResult();

    public boolean needsUpdate;

    {
        for (int i = 0; i < VORONOI_SIMPLEX_MAX_VERTICES; i++) {
            simplexVectorW[i] = new Vector3d();
            simplexPointsP[i] = new Vector3d();
            simplexPointsQ[i] = new Vector3d();
        }
    }

    public void removeVertex(int index) {
        assert (numVertices > 0);
        numVertices--;
        simplexVectorW[index].set(simplexVectorW[numVertices]);
        simplexPointsP[index].set(simplexPointsP[numVertices]);
        simplexPointsQ[index].set(simplexPointsQ[numVertices]);
    }

    public void reduceVertices(UsageBitfield usedVerts) {
        if ((numVertices() >= 4) && (!usedVerts.usedVertexD))
            removeVertex(3);

        if ((numVertices() >= 3) && (!usedVerts.usedVertexC))
            removeVertex(2);

        if ((numVertices() >= 2) && (!usedVerts.usedVertexB))
            removeVertex(1);

        if ((numVertices() >= 1) && (!usedVerts.usedVertexA))
            removeVertex(0);
    }

    @StaticAlloc
    public boolean updateClosestVectorAndPoints() {
        if (needsUpdate) {
            cachedBC.reset();

            needsUpdate = false;

            switch (numVertices()) {
                case 0:
                    cachedValidClosest = false;
                    break;
                case 1: {
                    cachedP1.set(simplexPointsP[0]);
                    cachedP2.set(simplexPointsQ[0]);
                    cachedV.sub(cachedP1, cachedP2); //== m_simplexVectorW[0]
                    cachedBC.reset();
                    cachedBC.setBarycentricCoordinates(1.0, 0.0, 0.0, 0.0);
                    cachedValidClosest = cachedBC.isValid();
                    break;
                }
                case 2: {
                    Vector3d tmp = Stack.newVec();

                    //closest point origin from line segment
                    Vector3d from = simplexVectorW[0];
                    Vector3d to = simplexVectorW[1];
                    Vector3d nearest = Stack.newVec();

                    Vector3d p = Stack.newVec();
                    p.set(0.0, 0.0, 0.0);
                    Vector3d diff = Stack.newVec();
                    diff.sub(p, from);

                    Vector3d v = Stack.newVec();
                    v.sub(to, from);

                    double t = v.dot(diff);

                    if (t > 0) {
                        double dotVV = v.dot(v);
                        if (t < dotVV) {
                            t /= dotVV;
                            tmp.scale(t, v);
                            diff.sub(tmp);
                            cachedBC.usedVertices.usedVertexA = true;
                        } else {
                            t = 1;
                            diff.sub(v);
                            // reduce to 1 point
                        }
                        cachedBC.usedVertices.usedVertexB = true;
                    } else {
                        t = 0;
                        //reduce to 1 point
                        cachedBC.usedVertices.usedVertexA = true;
                    }
                    cachedBC.setBarycentricCoordinates(1.0 - t, t, 0.0, 0.0);

                    tmp.scale(t, v);
                    nearest.add(from, tmp);

                    tmp.sub(simplexPointsP[1], simplexPointsP[0]);
                    tmp.scale(t);
                    cachedP1.add(simplexPointsP[0], tmp);

                    tmp.sub(simplexPointsQ[1], simplexPointsQ[0]);
                    tmp.scale(t);
                    cachedP2.add(simplexPointsQ[0], tmp);

                    cachedV.sub(cachedP1, cachedP2);

                    reduceVertices(cachedBC.usedVertices);

                    cachedValidClosest = cachedBC.isValid();
                    Stack.subVec(5);
                    break;
                }
                case 3: {
                    Vector3d tmp1 = Stack.newVec();
                    Vector3d tmp2 = Stack.newVec();
                    Vector3d tmp3 = Stack.newVec();

                    // closest point origin from triangle
                    Vector3d p = Stack.newVec();
                    p.set(0.0, 0.0, 0.0);

                    Vector3d a = simplexVectorW[0];
                    Vector3d b = simplexVectorW[1];
                    Vector3d c = simplexVectorW[2];

                    closestPtPointTriangle(p, a, b, c, cachedBC);

                    tmp1.scale(cachedBC.barycentricCoords[0], simplexPointsP[0]);
                    tmp2.scale(cachedBC.barycentricCoords[1], simplexPointsP[1]);
                    tmp3.scale(cachedBC.barycentricCoords[2], simplexPointsP[2]);
                    VectorUtil.add(cachedP1, tmp1, tmp2, tmp3);

                    tmp1.scale(cachedBC.barycentricCoords[0], simplexPointsQ[0]);
                    tmp2.scale(cachedBC.barycentricCoords[1], simplexPointsQ[1]);
                    tmp3.scale(cachedBC.barycentricCoords[2], simplexPointsQ[2]);
                    VectorUtil.add(cachedP2, tmp1, tmp2, tmp3);

                    cachedV.sub(cachedP1, cachedP2);

                    reduceVertices(cachedBC.usedVertices);
                    cachedValidClosest = cachedBC.isValid();
                    Stack.subVec(4);

                    break;
                }
                case 4: {

                    Vector3d a = simplexVectorW[0];
                    Vector3d b = simplexVectorW[1];
                    Vector3d c = simplexVectorW[2];
                    Vector3d d = simplexVectorW[3];

                    Vector3d p = Stack.newVec();
                    p.set(0.0, 0.0, 0.0);
                    boolean hasSeparation = closestPtPointTetrahedron(p, a, b, c, d, cachedBC);
                    Stack.subVec(1);

                    if (hasSeparation) {
                        Vector3d tmp1 = Stack.newVec();
                        Vector3d tmp2 = Stack.newVec();
                        Vector3d tmp3 = Stack.newVec();
                        Vector3d tmp4 = Stack.newVec();
                        tmp1.scale(cachedBC.barycentricCoords[0], simplexPointsP[0]);
                        tmp2.scale(cachedBC.barycentricCoords[1], simplexPointsP[1]);
                        tmp3.scale(cachedBC.barycentricCoords[2], simplexPointsP[2]);
                        tmp4.scale(cachedBC.barycentricCoords[3], simplexPointsP[3]);
                        VectorUtil.add(cachedP1, tmp1, tmp2, tmp3, tmp4);

                        tmp1.scale(cachedBC.barycentricCoords[0], simplexPointsQ[0]);
                        tmp2.scale(cachedBC.barycentricCoords[1], simplexPointsQ[1]);
                        tmp3.scale(cachedBC.barycentricCoords[2], simplexPointsQ[2]);
                        tmp4.scale(cachedBC.barycentricCoords[3], simplexPointsQ[3]);
                        VectorUtil.add(cachedP2, tmp1, tmp2, tmp3, tmp4);

                        cachedV.sub(cachedP1, cachedP2);
                        reduceVertices(cachedBC.usedVertices);
                        Stack.subVec(4);
                    } else {
                        //					printf("sub distance got penetration\n");
                        if (cachedBC.degenerate) {
                            cachedValidClosest = false;
                        } else {
                            cachedValidClosest = true;
                            //degenerate case == false, penetration = true + zero
                            cachedV.set(0.0, 0.0, 0.0);
                        }
                        break;
                    }

                    cachedValidClosest = cachedBC.isValid();

                    //closest point origin from tetrahedron
                    break;
                }
                default: {
                    cachedValidClosest = false;
                }
            }
        }

        return cachedValidClosest;
    }

    @StaticAlloc
    public boolean closestPtPointTriangle(Vector3d p, Vector3d a, Vector3d b, Vector3d c, SubSimplexClosestResult result) {
        result.usedVertices.reset();

        // Check if P in vertex region outside A
        Vector3d ab = Stack.newVec();
        ab.sub(b, a);

        Vector3d ac = Stack.newVec();
        ac.sub(c, a);

        Vector3d ap = Stack.newVec();
        ap.sub(p, a);

        double d1 = ab.dot(ap);
        double d2 = ac.dot(ap);

        if (d1 <= 0.0 && d2 <= 0.0) {
            result.closestPointOnSimplex.set(a);
            result.usedVertices.usedVertexA = true;
            result.setBarycentricCoordinates(1.0, 0.0, 0.0, 0.0);
            Stack.subVec(3);
            return true; // a; // barycentric coordinates (1,0,0)
        }

        // Check if P in vertex region outside B
        Vector3d bp = Stack.newVec();
        bp.sub(p, b);

        double d3 = ab.dot(bp);
        double d4 = ac.dot(bp);

        if (d3 >= 0.0 && d4 <= d3) {
            result.closestPointOnSimplex.set(b);
            result.usedVertices.usedVertexB = true;
            result.setBarycentricCoordinates(0, 1.0, 0.0, 0.0);
            Stack.subVec(4);
            return true; // b; // barycentric coordinates (0,1,0)
        }

        // Check if P in edge region of AB, if so return projection of P onto AB
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double v = d1 / (d1 - d3);
            result.closestPointOnSimplex.scaleAdd(v, ab, a);
            result.usedVertices.usedVertexA = true;
            result.usedVertices.usedVertexB = true;
            result.setBarycentricCoordinates(1.0 - v, v, 0.0, 0.0);
            Stack.subVec(4);
            return true;
            //return a + v * ab; // barycentric coordinates (1-v,v,0)
        }

        // Check if P in vertex region outside C
        Vector3d cp = Stack.newVec();
        cp.sub(p, c);

        double d5 = ab.dot(cp);
        double d6 = ac.dot(cp);

        if (d6 >= 0.0 && d5 <= d6) {
            result.closestPointOnSimplex.set(c);
            result.usedVertices.usedVertexC = true;
            result.setBarycentricCoordinates(0.0, 0.0, 1.0, 0.0);
            Stack.subVec(5);
            return true;//c; // barycentric coordinates (0,0,1)
        }

        // Check if P in edge region of AC, if so return projection of P onto AC
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double w = d2 / (d2 - d6);
            result.closestPointOnSimplex.scaleAdd(w, ac, a);
            result.usedVertices.usedVertexA = true;
            result.usedVertices.usedVertexC = true;
            result.setBarycentricCoordinates(1.0 - w, 0.0, w, 0.0);
            Stack.subVec(5);
            return true;
            //return a + w * ac; // barycentric coordinates (1-w,0,w)
        }

        // Check if P in edge region of BC, if so return projection of P onto BC
        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

            Vector3d tmp = Stack.newVec();
            tmp.sub(c, b);
            result.closestPointOnSimplex.scaleAdd(w, tmp, b);

            result.usedVertices.usedVertexB = true;
            result.usedVertices.usedVertexC = true;
            result.setBarycentricCoordinates(0, 1.0 - w, w, 0.0);
            Stack.subVec(6);
            return true;
            // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
        }

        // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        tmp1.scale(v, ab);
        tmp2.scale(w, ac);
        VectorUtil.add(result.closestPointOnSimplex, a, tmp1, tmp2);
        result.usedVertices.usedVertexA = true;
        result.usedVertices.usedVertexB = true;
        result.usedVertices.usedVertexC = true;
        result.setBarycentricCoordinates(1.0 - v - w, v, w, 0.0);
        Stack.subVec(7);

        return true;
        //	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = btScalar(1.0) - v - w
    }

    // Test if point p and d lie on opposite sides of plane through abc
    @StaticAlloc
    public static int pointOutsideOfPlane(Vector3d p, Vector3d a, Vector3d b, Vector3d c, Vector3d d) {
        Vector3d tmp = Stack.newVec();
        Vector3d normal = Stack.newVec();
        normal.sub(b, a);
        tmp.sub(c, a);
        normal.cross(normal, tmp);

        tmp.sub(p, a);
        double signp = tmp.dot(normal); // [AP AB AC]

        tmp.sub(d, a);
        double signd = tmp.dot(normal); // [AD AB AC]

        //#ifdef CATCH_DEGENERATE_TETRAHEDRON
//	#ifdef BT_USE_DOUBLE_PRECISION
//	if (signd * signd < (btScalar(1e-8) * btScalar(1e-8)))
//		{
//			return -1;
//		}
//	#else
        Stack.subVec(2);
        if (signd * signd < ((1e-4f) * (1e-4f))) {
            //		printf("affine dependent/degenerate\n");//
            return -1;
        }
        //#endif

        //#endif
        // Points on opposite sides if expression signs are opposite
        return (signp * signd < 0.0) ? 1 : 0;
    }

    @StaticAlloc
    public boolean closestPtPointTetrahedron(Vector3d p, Vector3d a, Vector3d b, Vector3d c, Vector3d d, SubSimplexClosestResult finalResult) {
        SubSimplexClosestResult tempResult = subsimplexResultsPool.get();
        tempResult.reset();
        try {
            Vector3d tmp = Stack.newVec();
            Vector3d q = Stack.newVec();

            // Start out assuming point inside all halfspaces, so closest to itself
            finalResult.closestPointOnSimplex.set(p);
            finalResult.usedVertices.reset();
            finalResult.usedVertices.usedVertexA = true;
            finalResult.usedVertices.usedVertexB = true;
            finalResult.usedVertices.usedVertexC = true;
            finalResult.usedVertices.usedVertexD = true;

            int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
            int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
            int pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
            int pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

            if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0) {
                finalResult.degenerate = true;
                return false;
            }

            if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0) {
                return false;
            }


            double bestSqDist = Float.MAX_VALUE;
            // If point outside face abc then compute closest point on abc
            if (pointOutsideABC != 0) {
                closestPtPointTriangle(p, a, b, c, tempResult);
                q.set(tempResult.closestPointOnSimplex);

                tmp.sub(q, p);
                double sqDist = tmp.dot(tmp);
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.closestPointOnSimplex.set(q);
                    //convert result bitmask!
                    finalResult.usedVertices.reset();
                    finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                    finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexB;
                    finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
                    finalResult.setBarycentricCoordinates(
                            tempResult.barycentricCoords[VERTEX_A],
                            tempResult.barycentricCoords[VERTEX_B],
                            tempResult.barycentricCoords[VERTEX_C],
                            0
                    );

                }
            }


            // Repeat test for face acd
            if (pointOutsideACD != 0) {
                closestPtPointTriangle(p, a, c, d, tempResult);
                q.set(tempResult.closestPointOnSimplex);
                //convert result bitmask!

                tmp.sub(q, p);
                double sqDist = tmp.dot(tmp);
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.closestPointOnSimplex.set(q);
                    finalResult.usedVertices.reset();
                    finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;

                    finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexB;
                    finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexC;
                    finalResult.setBarycentricCoordinates(
                            tempResult.barycentricCoords[VERTEX_A],
                            0,
                            tempResult.barycentricCoords[VERTEX_B],
                            tempResult.barycentricCoords[VERTEX_C]
                    );

                }
            }
            // Repeat test for face adb


            if (pointOutsideADB != 0) {
                closestPtPointTriangle(p, a, d, b, tempResult);
                q.set(tempResult.closestPointOnSimplex);
                //convert result bitmask!

                tmp.sub(q, p);
                double sqDist = tmp.dot(tmp);
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.closestPointOnSimplex.set(q);
                    finalResult.usedVertices.reset();
                    finalResult.usedVertices.usedVertexA = tempResult.usedVertices.usedVertexA;
                    finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexC;

                    finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;
                    finalResult.setBarycentricCoordinates(
                            tempResult.barycentricCoords[VERTEX_A],
                            tempResult.barycentricCoords[VERTEX_C],
                            0,
                            tempResult.barycentricCoords[VERTEX_B]
                    );

                }
            }
            // Repeat test for face bdc


            if (pointOutsideBDC != 0) {
                closestPtPointTriangle(p, b, d, c, tempResult);
                q.set(tempResult.closestPointOnSimplex);
                //convert result bitmask!
                tmp.sub(q, p);
                double sqDist = tmp.dot(tmp);
                if (sqDist < bestSqDist) {
                    bestSqDist = sqDist;
                    finalResult.closestPointOnSimplex.set(q);
                    finalResult.usedVertices.reset();
                    //
                    finalResult.usedVertices.usedVertexB = tempResult.usedVertices.usedVertexA;
                    finalResult.usedVertices.usedVertexC = tempResult.usedVertices.usedVertexC;
                    finalResult.usedVertices.usedVertexD = tempResult.usedVertices.usedVertexB;

                    finalResult.setBarycentricCoordinates(
                            0,
                            tempResult.barycentricCoords[VERTEX_A],
                            tempResult.barycentricCoords[VERTEX_C],
                            tempResult.barycentricCoords[VERTEX_B]
                    );

                }
            }

            //help! we ended up full !

            if (finalResult.usedVertices.usedVertexA &&
                    finalResult.usedVertices.usedVertexB &&
                    finalResult.usedVertices.usedVertexC &&
                    finalResult.usedVertices.usedVertexD) {
                return true;
            }

            return true;
        } finally {
            subsimplexResultsPool.release(tempResult);
        }
    }

    /**
     * Clear the simplex, remove all the vertices.
     */
    public void reset() {
        cachedValidClosest = false;
        numVertices = 0;
        needsUpdate = true;
        lastW.set(1e30, 1e30, 1e30);
        cachedBC.reset();
    }

    public void addVertex(Vector3d w, Vector3d p, Vector3d q) {
        lastW.set(w);
        needsUpdate = true;

        simplexVectorW[numVertices].set(w);
        simplexPointsP[numVertices].set(p);
        simplexPointsQ[numVertices].set(q);

        numVertices++;
    }

    /**
     * Return/calculate the closest vertex.
     */
    public boolean closest(Vector3d v) {
        boolean succes = updateClosestVectorAndPoints();
        v.set(cachedV);
        return succes;
    }

    public double maxVertex() {
        int i, numverts = numVertices();
        double maxV = 0.0;
        for (i = 0; i < numverts; i++) {
            double curLen2 = simplexVectorW[i].lengthSquared();
            if (maxV < curLen2) {
                maxV = curLen2;
            }
        }
        return maxV;
    }

    public boolean fullSimplex() {
        return (numVertices == 4);
    }

    public int getSimplex(Vector3d[] pBuf, Vector3d[] qBuf, Vector3d[] yBuf) {
        for (int i = 0; i < numVertices(); i++) {
            yBuf[i].set(simplexVectorW[i]);
            pBuf[i].set(simplexPointsP[i]);
            qBuf[i].set(simplexPointsQ[i]);
        }
        return numVertices();
    }

    public boolean inSimplex(Vector3d w) {
        boolean found = false;
        int i, numverts = numVertices();
        //btScalar maxV = btScalar(0.);

        //w is in the current (reduced) simplex
        for (i = 0; i < numverts; i++) {
            if (simplexVectorW[i].equals(w)) {
                found = true;
            }
        }

        //check in case lastW is already removed
        if (w.equals(lastW)) {
            return true;
        }

        return found;
    }

    public void backup_closest(Vector3d v) {
        v.set(cachedV);
    }

    public boolean emptySimplex() {
        return (numVertices() == 0);
    }

    public void compute_points(Vector3d p1, Vector3d p2) {
        updateClosestVectorAndPoints();
        p1.set(cachedP1);
        p2.set(cachedP2);
    }

    public int numVertices() {
        return numVertices;
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class UsageBitfield {
        public boolean usedVertexA;
        public boolean usedVertexB;
        public boolean usedVertexC;
        public boolean usedVertexD;

        public void reset() {
            usedVertexA = false;
            usedVertexB = false;
            usedVertexC = false;
            usedVertexD = false;
        }
    }

    public static class SubSimplexClosestResult {
        public final Vector3d closestPointOnSimplex = new Vector3d();
        //MASK for m_usedVertices
        //stores the simplex vertex-usage, using the MASK,
        // if m_usedVertices & MASK then the related vertex is used
        public final UsageBitfield usedVertices = new UsageBitfield();
        public final double[] barycentricCoords = new double[4];
        public boolean degenerate;

        public void reset() {
            degenerate = false;
            setBarycentricCoordinates(0.0, 0.0, 0.0, 0.0);
            usedVertices.reset();
        }

        public boolean isValid() {
            return (barycentricCoords[0] >= 0.0) &&
                    (barycentricCoords[1] >= 0.0) &&
                    (barycentricCoords[2] >= 0.0) &&
                    (barycentricCoords[3] >= 0.0);
        }

        public void setBarycentricCoordinates(double a, double b, double c, double d) {
            barycentricCoords[0] = a;
            barycentricCoords[1] = b;
            barycentricCoords[2] = c;
            barycentricCoords[3] = d;
        }
    }

}
