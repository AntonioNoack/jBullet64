package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import kotlin.NotImplementedError;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * ConvexHullShape implements an implicit convex hull of an array of vertices.
 * Bullet provides a general and fast collision detector for convex shapes based
 * on GJK and EPA using localGetSupportingVertex.
 *
 * @author jezek2
 */
@SuppressWarnings("unused")
public class ConvexHullShape extends PolyhedralConvexShape {

    private final ArrayList<Vector3d> points;

    @SuppressWarnings("unused")
    public ConvexHullShape(ArrayList<Vector3d> points) {
        this.points = points;
        recalculateLocalAabb();
    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        localScaling.set(scaling);
        recalculateLocalAabb();
    }

    @SuppressWarnings("unused")
    public void addPoint(Vector3d point) {
        points.add(point);
        recalculateLocalAabb();
    }

    @SuppressWarnings("unused")
    public int getNumPoints() {
        return points.size();
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d supVec) {
        supVec.set(0.0, 0.0, 0.0);
        double newDot, maxDot = Double.NEGATIVE_INFINITY;

        Vector3d vtx = Stack.newVec();
        for (Vector3d point : points) {
            VectorUtil.mul(vtx, point, localScaling);

            newDot = dir.dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                supVec.set(vtx);
            }
        }

        Stack.subVec(1);

        return supVec;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        double newDot;

        // JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
        // TODO: optimize it
        double[] wCoords = new double[numVectors];

        // use 'w' component of supportVerticesOut?
        for (int i = 0; i < numVectors; i++) {
            //supportVerticesOut[i][3] = btScalar(Double.NEGATIVE_INFINITY);
            wCoords[i] = Double.NEGATIVE_INFINITY;
        }

        Vector3d vtx = Stack.newVec();
        for (int i = 0, l = points.size(); i < l; i++) {
            Vector3d point = points.get(i);
            VectorUtil.mul(vtx, point, localScaling);

            for (int j = 0; j < numVectors; j++) {
                Vector3d vec = vectors[j];
                newDot = vec.dot(vtx);
                //if (newDot > supportVerticesOut[j][3])
                if (newDot > wCoords[j]) {
                    // WARNING: don't swap next lines, the w component would get overwritten!
                    supportVerticesOut[j].set(vtx);
                    //supportVerticesOut[j][3] = newDot;
                    wCoords[j] = newDot;
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
                vecNorm.set(-1, -1, -1);
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
        VectorUtil.mul(pa, points.get(index0), localScaling);
        VectorUtil.mul(pb, points.get(index1), localScaling);
    }

    @Override
    public void getVertex(int i, Vector3d vtx) {
        VectorUtil.mul(vtx, points.get(i), localScaling);
    }

    @Override
    public int getNumPlanes() {
        return 0;
    }

    @Override
    public void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i) {
        throw new NotImplementedError();
    }

    @Override
    public boolean isInside(Vector3d pt, double tolerance) {
        throw new NotImplementedError();
    }

    @Override
    public BroadphaseNativeType getShapeType() {
        return BroadphaseNativeType.CONVEX_HULL_SHAPE_PROXYTYPE;
    }

    @Override
    public String getName() {
        return "Convex";
    }

}
