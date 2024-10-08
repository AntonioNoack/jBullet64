package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.convexhull.HullDesc;
import com.bulletphysics.linearmath.convexhull.HullFlags;
import com.bulletphysics.linearmath.convexhull.HullLibrary;
import com.bulletphysics.linearmath.convexhull.HullResult;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ShapeHull takes a {@link ConvexShape}, builds the convex hull using {@link HullLibrary}
 * and provides triangle indices and vertices.
 *
 * @author jezek2
 */
public class ShapeHull {

    protected ObjectArrayList<Vector3d> vertices = new ObjectArrayList<Vector3d>();
    protected IntArrayList indices = new IntArrayList();
    protected int numIndices;
    protected ConvexShape shape;

    protected ObjectArrayList<Vector3d> unitSpherePoints = new ObjectArrayList<Vector3d>();

    public ShapeHull(ConvexShape shape) {
        this.shape = shape;
        this.vertices.clear();
        this.indices.clear();
        this.numIndices = 0;

        MiscUtil.resize(unitSpherePoints, NUM_UNIT_SPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2, Vector3d.class);
        for (int i = 0; i < constUnitSpherePoints.size(); i++) {
            unitSpherePoints.getQuick(i).set(constUnitSpherePoints.getQuick(i));
        }
    }

    public boolean buildHull(double margin) {
        Vector3d norm = Stack.newVec();

        int numSampleDirections = NUM_UNIT_SPHERE_POINTS;
        {
            int numPDA = shape.getNumPreferredPenetrationDirections();
            if (numPDA != 0) {
                for (int i = 0; i < numPDA; i++) {
                    shape.getPreferredPenetrationDirection(i, norm);
                    unitSpherePoints.getQuick(numSampleDirections).set(norm);
                    numSampleDirections++;
                }
            }
        }

        ObjectArrayList<Vector3d> supportPoints = new ObjectArrayList<Vector3d>();
        MiscUtil.resize(supportPoints, NUM_UNIT_SPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2, Vector3d.class);

        for (int i = 0; i < numSampleDirections; i++) {
            shape.localGetSupportingVertex(unitSpherePoints.getQuick(i), supportPoints.getQuick(i));
        }

        HullDesc hd = new HullDesc();
        hd.flags = HullFlags.TRIANGLES;
        hd.vcount = numSampleDirections;

        //#ifdef BT_USE_DOUBLE_PRECISION
        //hd.mVertices = &supportPoints[0];
        //hd.mVertexStride = sizeof(btVector3);
        //#else
        hd.vertices = supportPoints;
        //hd.vertexStride = 3 * 4;
        //#endif

        HullLibrary hl = new HullLibrary();
        HullResult hr = new HullResult();
        if (!hl.createConvexHull(hd, hr)) {
            return false;
        }

        MiscUtil.resize(vertices, hr.numOutputVertices, Vector3d.class);

        for (int i = 0; i < hr.numOutputVertices; i++) {
            vertices.getQuick(i).set(hr.outputVertices.getQuick(i));
        }
        numIndices = hr.numIndices;
        MiscUtil.resize(indices, numIndices, 0);
        for (int i = 0; i < numIndices; i++) {
            indices.set(i, hr.indices.get(i));
        }

        // free temporary hull result that we just copied
        hl.releaseResult(hr);

        return true;
    }

    public int numTriangles() {
        return numIndices / 3;
    }

    public int numVertices() {
        return vertices.size();
    }

    public int numIndices() {
        return numIndices;
    }

    public ObjectArrayList<Vector3d> getVertexPointer() {
        return vertices;
    }

    public IntArrayList getIndexPointer() {
        return indices;
    }

    ////////////////////////////////////////////////////////////////////////////

    private static final int NUM_UNIT_SPHERE_POINTS = 42;

    private static ObjectArrayList<Vector3d> constUnitSpherePoints = new ObjectArrayList<Vector3d>();

    static {
        constUnitSpherePoints.add(new Vector3d(+0.000000, -0.000000, -1.000000));
        constUnitSpherePoints.add(new Vector3d(+0.723608, -0.525725, -0.447219));
        constUnitSpherePoints.add(new Vector3d(-0.276388, -0.850649, -0.447219));
        constUnitSpherePoints.add(new Vector3d(-0.894426, -0.000000, -0.447216));
        constUnitSpherePoints.add(new Vector3d(-0.276388, +0.850649, -0.447220));
        constUnitSpherePoints.add(new Vector3d(+0.723608, +0.525725, -0.447219));
        constUnitSpherePoints.add(new Vector3d(+0.276388, -0.850649, +0.447220));
        constUnitSpherePoints.add(new Vector3d(-0.723608, -0.525725, +0.447219));
        constUnitSpherePoints.add(new Vector3d(-0.723608, +0.525725, +0.447219));
        constUnitSpherePoints.add(new Vector3d(+0.276388, +0.850649, +0.447219));
        constUnitSpherePoints.add(new Vector3d(+0.894426, +0.000000, +0.447216));
        constUnitSpherePoints.add(new Vector3d(-0.000000, +0.000000, +1.000000));
        constUnitSpherePoints.add(new Vector3d(+0.425323, -0.309011, -0.850654));
        constUnitSpherePoints.add(new Vector3d(-0.162456, -0.499995, -0.850654));
        constUnitSpherePoints.add(new Vector3d(+0.262869, -0.809012, -0.525738));
        constUnitSpherePoints.add(new Vector3d(+0.425323, +0.309011, -0.850654));
        constUnitSpherePoints.add(new Vector3d(+0.850648, -0.000000, -0.525736));
        constUnitSpherePoints.add(new Vector3d(-0.525730, -0.000000, -0.850652));
        constUnitSpherePoints.add(new Vector3d(-0.688190, -0.499997, -0.525736));
        constUnitSpherePoints.add(new Vector3d(-0.162456, +0.499995, -0.850654));
        constUnitSpherePoints.add(new Vector3d(-0.688190, +0.499997, -0.525736));
        constUnitSpherePoints.add(new Vector3d(+0.262869, +0.809012, -0.525738));
        constUnitSpherePoints.add(new Vector3d(+0.951058, +0.309013, +0.000000));
        constUnitSpherePoints.add(new Vector3d(+0.951058, -0.309013, +0.000000));
        constUnitSpherePoints.add(new Vector3d(+0.587786, -0.809017, +0.000000));
        constUnitSpherePoints.add(new Vector3d(+0.000000, -1.000000, +0.000000));
        constUnitSpherePoints.add(new Vector3d(-0.587786, -0.809017, +0.000000));
        constUnitSpherePoints.add(new Vector3d(-0.951058, -0.309013, -0.000000));
        constUnitSpherePoints.add(new Vector3d(-0.951058, +0.309013, -0.000000));
        constUnitSpherePoints.add(new Vector3d(-0.587786, +0.809017, -0.000000));
        constUnitSpherePoints.add(new Vector3d(-0.000000, +1.000000, -0.000000));
        constUnitSpherePoints.add(new Vector3d(+0.587786, +0.809017, -0.000000));
        constUnitSpherePoints.add(new Vector3d(+0.688190, -0.499997, +0.525736));
        constUnitSpherePoints.add(new Vector3d(-0.262869, -0.809012, +0.525738));
        constUnitSpherePoints.add(new Vector3d(-0.850648, +0.000000, +0.525736));
        constUnitSpherePoints.add(new Vector3d(-0.262869, +0.809012, +0.525738));
        constUnitSpherePoints.add(new Vector3d(+0.688190, +0.499997, +0.525736));
        constUnitSpherePoints.add(new Vector3d(+0.525730, +0.000000, +0.850652));
        constUnitSpherePoints.add(new Vector3d(+0.162456, -0.499995, +0.850654));
        constUnitSpherePoints.add(new Vector3d(-0.425323, -0.309011, +0.850654));
        constUnitSpherePoints.add(new Vector3d(-0.425323, +0.309011, +0.850654));
        constUnitSpherePoints.add(new Vector3d(+0.162456, +0.499995, +0.850654));
    }

}
