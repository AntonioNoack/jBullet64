package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import kotlin.NotImplementedError;

import javax.vecmath.Vector3d;

/**
 * ConvexHullShape implements an implicit convex hull of an array of vertices.
 * Bullet provides a general and fast collision detector for convex shapes based
 * on GJK and EPA using localGetSupportingVertex.
 *
 * @author Antonio, jezek2
 */
@SuppressWarnings("unused")
public class ConvexHullShape3 extends PolyhedralConvexShape {

    private final float[] points;
    private final int length;

    @SuppressWarnings("unused")
    public ConvexHullShape3(float[] points) {
        this.points = points;
        this.length = points.length / 3;
        recalculateLocalAabb();
    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        localScaling.set(scaling);
        recalculateLocalAabb();
    }

    @SuppressWarnings("unused")
    public int getNumPoints() {
        return length;
    }

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d supVec) {
        supVec.set(0.0, 0.0, 0.0);
        double newDot, maxDot = Double.NEGATIVE_INFINITY;

        for (int i = 0, l = points.length; i < l; i += 3) {
            double x = points[i] * localScaling.x;
            double y = points[i + 1] * localScaling.y;
            double z = points[i + 2] * localScaling.z;
            newDot = dir.x * x + dir.y * y + dir.z * z;
            if (newDot > maxDot) {
                maxDot = newDot;
                supVec.set(x, y, z);
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
            wCoords[i] = Double.NEGATIVE_INFINITY;
        }

        for (int i = 0, l = points.length; i < l; i += 3) {
            double x = points[i] * localScaling.x;
            double y = points[i + 1] * localScaling.y;
            double z = points[i + 2] * localScaling.z;

            for (int j = 0; j < numVectors; j++) {
                Vector3d vec = vectors[j];
                newDot = vec.x * x + vec.y * y + vec.z * z;
                if (newDot > wCoords[j]) {
                    // WARNING: don't swap next lines, the w component would get overwritten!
                    supportVerticesOut[j].set(x, y, z);
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
        return length;
    }

    @Override
    public int getNumEdges() {
        return length;
    }

    @Override
    public void getEdge(int i, Vector3d pa, Vector3d pb) {
        int index0 = (i % length) * 3;
        int index1 = ((i + 1) % length) * 3;
        pa.set(points[index0], points[index0 + 1], points[index0 + 2]);
        pb.set(points[index1], points[index1 + 1], points[index1 + 2]);
        VectorUtil.mul(pa, pa, localScaling);
        VectorUtil.mul(pb, pb, localScaling);
    }

    @Override
    public void getVertex(int i, Vector3d vtx) {
        i *= 3;
        vtx.x = points[i] * localScaling.x;
        vtx.y = points[i + 1] * localScaling.y;
        vtx.z = points[i + 2] * localScaling.z;
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
