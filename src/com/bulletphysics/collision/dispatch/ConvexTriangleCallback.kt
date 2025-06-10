package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * For each triangle in the concave mesh that overlaps with the AABB of a convex
 * (see {@link #convexBody} field), processTriangle is called.
 *
 * @author jezek2
 */
class ConvexTriangleCallback implements TriangleCallback {

    private final CollisionObject convexBody;
    private final CollisionObject triBody;

    private final Vector3d aabbMin = new Vector3d();
    private final Vector3d aabbMax = new Vector3d();

    private ManifoldResult resultOut;

    private final Dispatcher dispatcher;
    private DispatcherInfo dispatchInfoPtr;
    private double collisionMarginTriangle;

    public PersistentManifold manifoldPtr;

    public ConvexTriangleCallback(Dispatcher dispatcher, CollisionObject body0, CollisionObject body1, boolean isSwapped) {
        this.dispatcher = dispatcher;
        this.dispatchInfoPtr = null;

        convexBody = isSwapped ? body1 : body0;
        triBody = isSwapped ? body0 : body1;

        //
        // create the manifold from the dispatcher 'manifold pool'
        //
        manifoldPtr = dispatcher.getNewManifold(convexBody, triBody);

        clearCache();
    }

    public void destroy() {
        clearCache();
        dispatcher.releaseManifold(manifoldPtr);
    }

    public void setTimeStepAndCounters(double collisionMarginTriangle, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        this.dispatchInfoPtr = dispatchInfo;
        this.collisionMarginTriangle = collisionMarginTriangle;
        this.resultOut = resultOut;

        // recalc aabbs
        Transform convexInTriangleSpace = Stack.newTrans();

        triBody.getWorldTransform(convexInTriangleSpace);
        convexInTriangleSpace.inverse();
        convexInTriangleSpace.mul(convexBody.getWorldTransform(Stack.newTrans()));

        CollisionShape convexShape = convexBody.getCollisionShape();
        convexShape.getAabb(convexInTriangleSpace, aabbMin, aabbMax);
        Vector3d extra = Stack.newVec();
        extra.set(collisionMarginTriangle, collisionMarginTriangle, collisionMarginTriangle);

        aabbMax.add(extra);
        aabbMin.sub(extra);
    }

    private final CollisionAlgorithmConstructionInfo ci = new CollisionAlgorithmConstructionInfo();
    private final TriangleShape tm = new TriangleShape();

    public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {

        // aabb filter is already applied!

        ci.dispatcher1 = dispatcher;

        CollisionObject ob = triBody;

        // debug drawing of the overlapping triangles
        if (dispatchInfoPtr != null && dispatchInfoPtr.debugDraw != null && dispatchInfoPtr.debugDraw.getDebugMode() > 0) {
            Vector3d color = Stack.newVec();
            color.set(255, 255, 0);
            Transform tr = ob.getWorldTransform(Stack.newTrans());

            Vector3d tmp1 = Stack.newVec();
            Vector3d tmp2 = Stack.newVec();

            tmp1.set(triangle[0]);
            tr.transform(tmp1);
            tmp2.set(triangle[1]);
            tr.transform(tmp2);
            dispatchInfoPtr.debugDraw.drawLine(tmp1, tmp2, color);

            tmp1.set(triangle[1]);
            tr.transform(tmp1);
            tmp2.set(triangle[2]);
            tr.transform(tmp2);
            dispatchInfoPtr.debugDraw.drawLine(tmp1, tmp2, color);

            tmp1.set(triangle[2]);
            tr.transform(tmp1);
            tmp2.set(triangle[0]);
            tr.transform(tmp2);
            dispatchInfoPtr.debugDraw.drawLine(tmp1, tmp2, color);
        }

        if (convexBody.getCollisionShape().isConvex()) {
            tm.init(triangle[0], triangle[1], triangle[2]);
            tm.setMargin(collisionMarginTriangle);

            CollisionShape tmpShape = ob.getCollisionShape();
            ob.internalSetTemporaryCollisionShape(tm);

            CollisionAlgorithm colAlgo = ci.dispatcher1.findAlgorithm(convexBody, triBody, manifoldPtr);

            resultOut.setShapeIdentifiers(-1, -1, partId, triangleIndex);
            colAlgo.processCollision(convexBody, triBody, dispatchInfoPtr, resultOut);
            ci.dispatcher1.freeCollisionAlgorithm(colAlgo);
            ob.internalSetTemporaryCollisionShape(tmpShape);
        }
    }

    public void clearCache() {
        dispatcher.clearManifold(manifoldPtr);
    }

    public Vector3d getAabbMin(Vector3d out) {
        out.set(aabbMin);
        return out;
    }

    public Vector3d getAabbMax(Vector3d out) {
        out.set(aabbMax);
        return out;
    }

}
