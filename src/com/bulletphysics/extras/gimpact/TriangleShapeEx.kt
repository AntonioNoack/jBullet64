package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * @author jezek2
 */
public class TriangleShapeEx extends TriangleShape {

    public TriangleShapeEx() {
        super();
    }

    public TriangleShapeEx(Vector3d p0, Vector3d p1, Vector3d p2) {
        super(p0, p1, p2);
    }

    @Override
    public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        Vector3d tv0 = Stack.newVec(vertices1[0]);
        t.transform(tv0);
        Vector3d tv1 = Stack.newVec(vertices1[1]);
        t.transform(tv1);
        Vector3d tv2 = Stack.newVec(vertices1[2]);
        t.transform(tv2);

        AABB triangleBox = new AABB();
        triangleBox.init(tv0, tv1, tv2, collisionMargin);

        aabbMin.set(triangleBox.min);
        aabbMax.set(triangleBox.max);

        Stack.subVec(3);
    }

    public void applyTransform(Transform t) {
        t.transform(vertices1[0]);
        t.transform(vertices1[1]);
        t.transform(vertices1[2]);
    }

    public void buildTriPlane(Vector4d plane) {
        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        Vector3d normal = Stack.newVec();
        tmp1.sub(vertices1[1], vertices1[0]);
        tmp2.sub(vertices1[2], vertices1[0]);
        normal.cross(tmp1, tmp2);
        normal.normalize();

        plane.set(normal.x, normal.y, normal.z, vertices1[0].dot(normal));
    }

    public boolean overlapTestConservative(TriangleShapeEx other) {
        double total_margin = getMargin() + other.getMargin();

        Vector4d plane0 = new Vector4d();
        buildTriPlane(plane0);
        Vector4d plane1 = new Vector4d();
        other.buildTriPlane(plane1);

        // classify points on other triangle
        double dis0 = ClipPolygon.distancePointPlane(plane0, other.vertices1[0]) - total_margin;

        double dis1 = ClipPolygon.distancePointPlane(plane0, other.vertices1[1]) - total_margin;

        double dis2 = ClipPolygon.distancePointPlane(plane0, other.vertices1[2]) - total_margin;

        if (dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0) {
            return false; // classify points on this triangle
        }
        dis0 = ClipPolygon.distancePointPlane(plane1, vertices1[0]) - total_margin;

        dis1 = ClipPolygon.distancePointPlane(plane1, vertices1[1]) - total_margin;

        dis2 = ClipPolygon.distancePointPlane(plane1, vertices1[2]) - total_margin;

        return !(dis0 > 0.0) || !(dis1 > 0.0) || !(dis2 > 0.0);
    }

}
