package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * PolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
 *
 * @author jezek2
 */
public abstract class PolyhedralConvexShape extends ConvexInternalShape {

    private static final Vector3d[] directions = new Vector3d[]{
            new Vector3d(1.0, 0.0, 0.0),
            new Vector3d(0.0, 1.0, 0.0),
            new Vector3d(0.0, 0.0, 1.0),
            new Vector3d(-1.0, 0.0, 0.0),
            new Vector3d(0.0, -1.0, 0.0),
            new Vector3d(0.0, 0.0, -1.0)
    };

    private static final Vector3d[] supporting = new Vector3d[]{
            new Vector3d(0.0, 0.0, 0.0),
            new Vector3d(0.0, 0.0, 0.0),
            new Vector3d(0.0, 0.0, 0.0),
            new Vector3d(0.0, 0.0, 0.0),
            new Vector3d(0.0, 0.0, 0.0),
            new Vector3d(0.0, 0.0, 0.0)
    };

    protected final Vector3d localAabbMin = new Vector3d(1.0, 1.0, 1.0);
    protected final Vector3d localAabbMax = new Vector3d(-1.0, -1.0, -1.0);
    protected boolean isLocalAabbValid = false;

//	/** optional Hull is for optional Separating Axis Test Hull collision detection, see Hull.cpp */
//	public Hull optionalHull = null;

    @Override
    public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
        int i;
        out.set(0.0, 0.0, 0.0);

        double maxDot = Double.NEGATIVE_INFINITY;

        Vector3d vec = Stack.newVec(dir);
        double lenSqr = vec.lengthSquared();
        if (lenSqr < 0.0001) {
            vec.set(1.0, 0.0, 0.0);
        } else {
            double invLen = 1.0 / Math.sqrt(lenSqr);
            vec.scale(invLen);
        }

        Vector3d vtx = Stack.newVec();
        double newDot;

        for (i = 0; i < getNumVertices(); i++) {
            getVertex(i, vtx);
            newDot = vec.dot(vtx);
            if (newDot > maxDot) {
                maxDot = newDot;
                // why was this removed???
                out.set(vtx);
            }
        }

        Stack.subVec(2);

        return out;
    }

    @Override
    public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
        int i;

        Vector3d vtx = Stack.newVec();
        double newDot;

        // JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
        // TODO: optimize it
        double[] wcoords = new double[numVectors];

        for (i = 0; i < numVectors; i++) {
            // TODO: used w in vector3:
            //supportVerticesOut[i].w = Double.NEGATIVE_INFINITY;
            wcoords[i] = Double.NEGATIVE_INFINITY;
        }

        for (int j = 0; j < numVectors; j++) {
            Vector3d vec = vectors[j];

            for (i = 0; i < getNumVertices(); i++) {
                getVertex(i, vtx);
                newDot = vec.dot(vtx);
                //if (newDot > supportVerticesOut[j].w)
                if (newDot > wcoords[j]) {
                    //WARNING: don't swap next lines, the w component would get overwritten!
                    supportVerticesOut[j].set(vtx);
                    //supportVerticesOut[j].w = newDot;
                    wcoords[j] = newDot;
                }
            }
        }
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        // not yet, return box inertia

        double margin = getMargin();

        Transform identity = Stack.newTrans();
        identity.setIdentity();
        Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
        getAabb(identity, aabbMin, aabbMax);

        Vector3d halfExtents = Stack.newVec();
        halfExtents.sub(aabbMax, aabbMin);
        halfExtents.scale(0.5);

        double lx = halfExtents.x + margin;
        double ly = halfExtents.y + margin;
        double lz = halfExtents.z + margin;
        double x2 = lx * lx;
        double y2 = ly * ly;
        double z2 = lz * lz;

        inertia.set(y2 + z2, x2 + z2, x2 + y2);
        inertia.scale(mass / 3.0);
    }

    private void getNonvirtualAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax, double margin) {
        // lazy evaluation of local aabb
        assert (isLocalAabbValid);

        AabbUtil2.transformAabb(localAabbMin, localAabbMax, margin, trans, aabbMin, aabbMax);
    }

    @Override
    public void getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
        getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
    }

    protected final void _PolyhedralConvexShape_getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
        getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
    }

    public void recalculateLocalAabb() {
        isLocalAabbValid = true;

        batchedUnitVectorGetSupportingVertexWithoutMargin(directions, supporting, 6);

        localAabbMax.x = supporting[0].x + collisionMargin;
        localAabbMin.x = supporting[3].x - collisionMargin;

        localAabbMax.y = supporting[1].y + collisionMargin;
        localAabbMin.y = supporting[4].y - collisionMargin;

        localAabbMax.z = supporting[2].z + collisionMargin;
        localAabbMin.z = supporting[5].z - collisionMargin;

    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        super.setLocalScaling(scaling);
        recalculateLocalAabb();
    }

    public abstract int getNumVertices();

    public abstract int getNumEdges();

    public abstract void getEdge(int i, Vector3d pa, Vector3d pb);

    public abstract void getVertex(int i, Vector3d vtx);

    public abstract int getNumPlanes();

    public abstract void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i);

//	public abstract  int getIndex(int i) const = 0 ; 

    public abstract boolean isInside(Vector3d pt, double tolerance);

}
