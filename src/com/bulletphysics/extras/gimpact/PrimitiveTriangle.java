package com.bulletphysics.extras.gimpact;

import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * @author jezek2
 */
public class PrimitiveTriangle {

    private final ObjectArrayList<Vector3d> tmpVecList1 = new ObjectArrayList<Vector3d>(TriangleContact.MAX_TRI_CLIPPING);
    private final ObjectArrayList<Vector3d> tmpVecList2 = new ObjectArrayList<Vector3d>(TriangleContact.MAX_TRI_CLIPPING);
    private final ObjectArrayList<Vector3d> tmpVecList3 = new ObjectArrayList<Vector3d>(TriangleContact.MAX_TRI_CLIPPING);

    {
        for (int i = 0; i < TriangleContact.MAX_TRI_CLIPPING; i++) {
            tmpVecList1.add(new Vector3d());
            tmpVecList2.add(new Vector3d());
            tmpVecList3.add(new Vector3d());
        }
    }

    public final Vector3d[] vertices = new Vector3d[3];
    public final Vector4d plane = new Vector4d();
    public double margin = 0.01;

    public PrimitiveTriangle() {
        for (int i = 0; i < vertices.length; i++) {
            vertices[i] = new Vector3d();
        }
    }

    public void set(PrimitiveTriangle tri) {
        throw new UnsupportedOperationException();
    }

    public void buildTriPlane() {
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        Vector3d normal = Stack.newVec();
        tmp1.sub(vertices[1], vertices[0]);
        tmp2.sub(vertices[2], vertices[0]);
        normal.cross(tmp1, tmp2);
        normal.normalize();

        plane.set(normal.x, normal.y, normal.z, vertices[0].dot(normal));
        Stack.subVec(3);
    }

    /**
     * Test if triangles could collide.
     */
    public boolean overlapTestConservative(PrimitiveTriangle other) {
        double total_margin = margin + other.margin;
        // classify points on other triangle
        double dis0 = ClipPolygon.distancePointPlane(plane, other.vertices[0]) - total_margin;
        double dis1 = ClipPolygon.distancePointPlane(plane, other.vertices[1]) - total_margin;
        double dis2 = ClipPolygon.distancePointPlane(plane, other.vertices[2]) - total_margin;

        if (dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0) {
            return false; // classify points on this triangle
        }

        dis0 = ClipPolygon.distancePointPlane(other.plane, vertices[0]) - total_margin;
        dis1 = ClipPolygon.distancePointPlane(other.plane, vertices[1]) - total_margin;
        dis2 = ClipPolygon.distancePointPlane(other.plane, vertices[2]) - total_margin;
        return !(dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0);
    }

    /**
     * Calculates the plane which is parallel to the edge and perpendicular to the triangle plane.
     * This triangle must have its plane calculated.
     */
    public void get_edge_plane(int edge_index, Vector4d plane) {
        Vector3d e0 = vertices[edge_index];
        Vector3d e1 = vertices[(edge_index + 1) % 3];

        Vector3d tmp = Stack.newVec();
        tmp.set(this.plane.x, this.plane.y, this.plane.z);

        GeometryOperations.edgePlane(e0, e1, tmp, plane);
    }

    public void applyTransform(Transform t) {
        t.transform(vertices[0]);
        t.transform(vertices[1]);
        t.transform(vertices[2]);
    }

    /**
     * Clips the triangle against this.
     *
     * @param clipped_points must have MAX_TRI_CLIPPING size, and this triangle must have its plane calculated.
     * @return the number of clipped points
     */
    public int clip_triangle(PrimitiveTriangle other, ObjectArrayList<Vector3d> clipped_points) {
        // edge 0
        ObjectArrayList<Vector3d> temp_points = tmpVecList1;

        Vector4d edgeplane = new Vector4d();

        get_edge_plane(0, edgeplane);

        int clipped_count = ClipPolygon.planeClipTriangle(edgeplane, other.vertices[0], other.vertices[1], other.vertices[2], temp_points);

        if (clipped_count == 0) {
            return 0;
        }
        ObjectArrayList<Vector3d> temp_points1 = tmpVecList2;

        // edge 1
        get_edge_plane(1, edgeplane);

        clipped_count = ClipPolygon.planeClipPolygon(edgeplane, temp_points, clipped_count, temp_points1);

        if (clipped_count == 0) {
            return 0; // edge 2
        }
        get_edge_plane(2, edgeplane);

        clipped_count = ClipPolygon.planeClipPolygon(edgeplane, temp_points1, clipped_count, clipped_points);

        return clipped_count;
    }

    /**
     * Find collision using the clipping method.
     * This triangle and other must have their triangles calculated.
     */
    public boolean findTriangleCollisionClipMethod(PrimitiveTriangle other, TriangleContact contacts) {
        double margin = this.margin + other.margin;

        ObjectArrayList<Vector3d> clipped_points = tmpVecList3;

        int clipped_count;
        //create planes
        // plane v vs U points

        TriangleContact contacts1 = new TriangleContact();

        contacts1.separatingNormal.set(plane);

        clipped_count = clip_triangle(other, clipped_points);

        if (clipped_count == 0) {
            return false; // Reject
        }

        // find most deep interval face1
        contacts1.mergePoints(contacts1.separatingNormal, margin, clipped_points, clipped_count);
        if (contacts1.pointCount == 0) {
            return false; // too far
            // Normal pointing to this triangle
        }
        contacts1.separatingNormal.x *= -1.0;
        contacts1.separatingNormal.y *= -1.0;
        contacts1.separatingNormal.z *= -1.0;

        // Clip tri1 by tri2 edges
        TriangleContact contacts2 = new TriangleContact();
        contacts2.separatingNormal.set(other.plane);

        clipped_count = other.clip_triangle(this, clipped_points);

        if (clipped_count == 0) {
            return false; // Reject
        }

        // find most deep interval face1
        contacts2.mergePoints(contacts2.separatingNormal, margin, clipped_points, clipped_count);
        if (contacts2.pointCount == 0) {
            return false; // too far

            // check most dir for contacts
        }
        if (contacts2.penetrationDepth < contacts1.penetrationDepth) {
            contacts.copyFrom(contacts2);
        } else {
            contacts.copyFrom(contacts1);
        }
        return true;
    }

}
