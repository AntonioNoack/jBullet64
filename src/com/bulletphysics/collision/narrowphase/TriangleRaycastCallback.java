package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public abstract class TriangleRaycastCallback extends TriangleCallback {

    //protected final BulletStack stack = BulletStack.get();

    public final Vector3d from = new Vector3d();
    public final Vector3d to = new Vector3d();

    public double hitFraction;

    public TriangleRaycastCallback(Vector3d from, Vector3d to) {
        this.from.set(from);
        this.to.set(to);
        this.hitFraction = 1.0;
    }

    public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
        Vector3d vert0 = triangle[0];
        Vector3d vert1 = triangle[1];
        Vector3d vert2 = triangle[2];

        Vector3d v10 = Stack.newVec();
        v10.sub(vert1, vert0);

        Vector3d v20 = Stack.newVec();
        v20.sub(vert2, vert0);

        Vector3d triangleNormal = Stack.newVec();
        triangleNormal.cross(v10, v20);

        double dist = vert0.dot(triangleNormal);
        double dist_a = triangleNormal.dot(from);
        dist_a -= dist;
        double dist_b = triangleNormal.dot(to);
        dist_b -= dist;

        if (dist_a * dist_b >= 0.0) {
            return; // same sign
        }

        double proj_length = dist_a - dist_b;
        double distance = (dist_a) / (proj_length);
        // Now we have the intersection point on the plane, we'll see if it's inside the triangle
        // Add an epsilon as a tolerance for the raycast,
        // in case the ray hits exacly on the edge of the triangle.
        // It must be scaled for the triangle size.

        if (distance < hitFraction) {
            double edge_tolerance = triangleNormal.lengthSquared();
            edge_tolerance *= -0.0001f;
            Vector3d point = Stack.newVec();
            VectorUtil.setInterpolate3(point, from, to, distance);
            {
                Vector3d v0p = Stack.newVec();
                v0p.sub(vert0, point);
                Vector3d v1p = Stack.newVec();
                v1p.sub(vert1, point);
                Vector3d cp0 = Stack.newVec();
                cp0.cross(v0p, v1p);

                if (cp0.dot(triangleNormal) >= edge_tolerance) {
                    Vector3d v2p = Stack.newVec();
                    v2p.sub(vert2, point);
                    Vector3d cp1 = Stack.newVec();
                    cp1.cross(v1p, v2p);
                    if (cp1.dot(triangleNormal) >= edge_tolerance) {
                        Vector3d cp2 = Stack.newVec();
                        cp2.cross(v2p, v0p);

                        if (cp2.dot(triangleNormal) >= edge_tolerance) {

                            if (dist_a > 0.0) {
                                hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex);
                            } else {
                                Vector3d tmp = Stack.newVec();
                                tmp.negate(triangleNormal);
                                hitFraction = reportHit(tmp, distance, partId, triangleIndex);
                            }
                        }
                    }
                }
            }
        }
    }

    public abstract double reportHit(Vector3d hitNormalLocal, double hitFraction, int partId, int triangleIndex);

}
