package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.shapes.TriangleCallback;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
class GImpactTriangleCallback implements TriangleCallback {

    public GImpactCollisionAlgorithm algorithm;
    public CollisionObject body0;
    public CollisionObject body1;
    public GImpactShapeInterface shape;
    public boolean swapped;
    public double margin;

    public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
        TriangleShapeEx tri1 = new TriangleShapeEx(triangle[0], triangle[1], triangle[2]);
        tri1.setMargin(margin);
        if (swapped) {
            algorithm.setPart0(partId);
            algorithm.setFace0(triangleIndex);
        } else {
            algorithm.setPart1(partId);
            algorithm.setFace1(triangleIndex);
        }
        algorithm.gimpactVsShape(body0, body1, shape, tri1, swapped);
    }

}
