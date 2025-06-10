package com.bulletphysics.extras.gimpact;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * @author jezek2
 */
class GeometryOperations {
    /**
     * Calc a plane from a triangle edge an a normal.
     */
    public static void edgePlane(Vector3d e1, Vector3d e2, Vector3d normal, Vector4d plane) {
        Vector3d planeNormal = Stack.newVec();
        planeNormal.sub(e2, e1);
        planeNormal.cross(planeNormal, normal);
        planeNormal.normalize();

        plane.set(planeNormal);
        plane.w = e2.dot(planeNormal);
    }
}
