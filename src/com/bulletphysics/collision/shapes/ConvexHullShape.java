package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;
import org.jetbrains.annotations.NotNull;

import javax.vecmath.Vector3d;
import java.util.Arrays;
import java.util.List;

/**
 * ConvexHullShape implements an implicit convex hull of an array of vertices.
 * Bullet provides a general and fast collision detector for convex shapes based
 * on GJK and EPA using localGetSupportingVertex.
 *
 * @author jezek2
 */
public class ConvexHullShape extends PolyhedralConvexShape {

    private final ObjectArrayList<Vector3d> points;

    /**
     * This constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive double (x,y,z), the striding defines the number of bytes between each point, in memory.
     * It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
     * ConvexHullShape make an internal copy of the points.
     */
    @SuppressWarnings("unused")
    public ConvexHullShape(List<Vector3d> points) {
        this.points = new ObjectArrayList<>(points.size());
        this.points.addAll(points);
        recalculateLocalAabb();
    }

    @Override
    public void setLocalScaling(@NotNull Vector3d scaling) {
        localScaling.set(scaling);
        recalculateLocalAabb();
    }

    @SuppressWarnings("unused")
    public void addPoint(Vector3d point) {
        points.add(new Vector3d(point));
        recalculateLocalAabb();
    }

    public ObjectArrayList<Vector3d> getPoints() {
        return points;
    }

    @SuppressWarnings("unused")
    public int getNumPoints() {
        return points.size();
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d vec0, Vector3d out) {
        out.set(0.0, 0.0, 0.0);
        double newDot, maxDot = -1e308;

        Vector3d vec = Stack.newVec(vec0);
        double lenSqr = vec.lengthSquared();
        if (lenSqr < 0.0001f) {
            vec.set(1.0, 0.0, 0.0);
        } else {
            double rlen = 1.0 / Math.sqrt(lenSqr);
            vec.scale(rlen);
        }


        Vector3d vtx = Stack.newVec();
        for (int i = 0; i < points.size(); i++) {
            VectorUtil.mul(vtx, points.getQuick(i), localScaling);

            newDot = vec.dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                out.set(vtx);
            }
        }
        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] dirs, Vector3d[] outs, int numVectors) {
        double newDot;

        // JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
        // TODO: optimize it
        double[] wcoords = new double[numVectors];

        // use 'w' component of supportVerticesOut?
        Arrays.fill(wcoords, -1e308);

        Vector3d vtx = Stack.newVec();
        for (int i = 0; i < points.size(); i++) {
            VectorUtil.mul(vtx, points.getQuick(i), localScaling);

            for (int j = 0; j < numVectors; j++) {
                Vector3d vec = dirs[j];

                newDot = vec.dot(vtx);
                //if (newDot > supportVerticesOut[j][3])
                if (newDot > wcoords[j]) {
                    // WARNING: don't swap next lines, the w component would get overwritten!
                    outs[j].set(vtx);
                    //supportVerticesOut[j][3] = newDot;
                    wcoords[j] = newDot;
                }
            }
        }
    }

    @Override
    public Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out) {
        Vector3d supVertex = localGetSupportingVertexWithoutMargin(dir, out);
        if (getMargin() != 0.0) {
            Vector3d vecNorm = Stack.newVec(dir);
            if (vecNorm.lengthSquared() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
                vecNorm.set(-1.0, -1.0, -1.0);
            }
            vecNorm.normalize();
            supVertex.scaleAdd(getMargin(), vecNorm, supVertex);
            Stack.subVec(1);
        }
        return out;
    }

    /**
     * Currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection.
     * Please note that you can debug-draw ConvexHullShape with the Raytracer Demo.
     */
    @Override
    public int getNumVertices() {
        return points.size();
    }

    @Override
    public int getNumEdges() {
        return points.size();
    }

    @Override
    public void getEdge(int i, Vector3d pa, Vector3d pb) {
        int index0 = i % points.size();
        int index1 = (i + 1) % points.size();
        VectorUtil.mul(pa, points.getQuick(index0), localScaling);
        VectorUtil.mul(pb, points.getQuick(index1), localScaling);
    }

    @Override
    public void getVertex(int i, Vector3d vtx) {
        VectorUtil.mul(vtx, points.getQuick(i), localScaling);
    }

    @Override
    public int getNumPlanes() {
        return 0;
    }

    @Override
    public void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i) {
        assert false;
    }

    @Override
    public boolean isInside(Vector3d pt, double tolerance) {
        assert false;
        return false;
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.CONVEX_HULL_SHAPE_PROXYTYPE;
    }
}
