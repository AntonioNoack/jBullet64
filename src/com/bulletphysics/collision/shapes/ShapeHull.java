/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * ShapeHull implemented by John McCutchan.
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.convexhull.HullDesc;
import com.bulletphysics.linearmath.convexhull.HullFlags;
import com.bulletphysics.linearmath.convexhull.HullLibrary;
import com.bulletphysics.linearmath.convexhull.HullResult;
import com.bulletphysics.util.IntArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * ShapeHull takes a {@link ConvexShape}, builds the convex hull using {@link HullLibrary}
 * and provides triangle indices and vertices.
 *
 * @author jezek2
 */
public class ShapeHull {

    public ArrayList<Vector3d> vertices = new ArrayList<>();
    public IntArrayList indices = new IntArrayList();
    public int numIndices;
    public ConvexShape shape;

    public ArrayList<Vector3d> unitSpherePoints = new ArrayList<>(SUM_PTS);

    public ShapeHull(ConvexShape shape) {
        this.shape = shape;
        this.vertices.clear();
        this.indices.clear();
        this.numIndices = 0;

        for (int i = 0; i < SUM_PTS; i++) {
            unitSpherePoints.add(new Vector3d());
        }
        for (int i = 0; i < constUnitSpherePoints.size(); i++) {
            unitSpherePoints.get(i).set(constUnitSpherePoints.get(i));
        }
    }

    @SuppressWarnings("unused")
    public boolean buildHull(double margin) {
        Vector3d norm = Stack.newVec();

        int numSampleDirections = NUM_UNITSPHERE_POINTS;
        {
            int numPDA = shape.getNumPreferredPenetrationDirections();
            if (numPDA != 0) {
                for (int i = 0; i < numPDA; i++) {
                    shape.getPreferredPenetrationDirection(i, norm);
                    unitSpherePoints.get(numSampleDirections).set(norm);
                    numSampleDirections++;
                }
            }
        }

        ArrayList<Vector3d> supportPoints = new ArrayList<>(SUM_PTS);
        for (int i = 0; i < SUM_PTS; i++) supportPoints.add(new Vector3d());

        for (int i = 0; i < numSampleDirections; i++) {
            shape.localGetSupportingVertex(unitSpherePoints.get(i), supportPoints.get(i));
        }

        HullDesc hd = new HullDesc();
        hd.flags = HullFlags.TRIANGLES;
        hd.vertexCount = numSampleDirections;

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
            vertices.get(i).set(hr.outputVertices.get(i));
        }
        numIndices = hr.numIndices;
        MiscUtil.resize(indices, numIndices);
        for (int i = 0; i < numIndices; i++) {
            indices.set(i, hr.indices.get(i));
        }

        // free temporary hull result that we just copied
        hl.releaseResult(hr);

        return true;
    }

    @SuppressWarnings("unused")
    public int numTriangles() {
        return numIndices / 3;
    }

    @SuppressWarnings("unused")
    public int numVertices() {
        return vertices.size();
    }

    @SuppressWarnings("unused")
    public int numIndices() {
        return numIndices;
    }

    @SuppressWarnings("unused")
    public ArrayList<Vector3d> getVertexPointer() {
        return vertices;
    }

    @SuppressWarnings("unused")
    public IntArrayList getIndexPointer() {
        return indices;
    }

    ////////////////////////////////////////////////////////////////////////////

    public static final int NUM_UNITSPHERE_POINTS = 42;
    public static final int SUM_PTS = NUM_UNITSPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2;

    public static final ArrayList<Vector3d> constUnitSpherePoints = new ArrayList<>();

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
