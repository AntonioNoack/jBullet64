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
class ConvexTriangleCallback extends TriangleCallback {

    //protected final BulletStack stack = BulletStack.get();

    private final CollisionObject convexBody;
    private final CollisionObject triBody;

    private final Vector3d aabbMin = new Vector3d();
    private final Vector3d aabbMax = new Vector3d();

    private ManifoldResult resultOut;

    private final Dispatcher dispatcher;
    private DispatcherInfo dispatchInfoPtr;
    private double collisionMarginTriangle;

    public int triangleCount;
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

        // recalculate aabbs
        Transform convexInTriangleSpace = Stack.newTrans();

        triBody.getWorldTransform(convexInTriangleSpace);
        convexInTriangleSpace.inverse();
        convexInTriangleSpace.mul(convexBody.getWorldTransform(Stack.newTrans()));

        CollisionShape convexShape = convexBody.getCollisionShape();
        //CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
        convexShape.getAabb(convexInTriangleSpace, aabbMin, aabbMax);
        Vector3d extra = Stack.borrowVec();
        extra.set(collisionMarginTriangle, collisionMarginTriangle, collisionMarginTriangle);

        aabbMax.add(extra);
        aabbMin.sub(extra);
        Stack.subTrans(2);
    }

    private final CollisionAlgorithmConstructionInfo ci = new CollisionAlgorithmConstructionInfo();
    private final TriangleShape tm = new TriangleShape();

    public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
        // just for debugging purposes
        //printf("triangle %d",m_triangleCount++);

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

            Stack.subVec(3);
            Stack.subTrans(1);

            //btVector3 center = triangle[0] + triangle[1]+triangle[2];
            //center *= btScalar(0.333333);
            //m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(center),color);
            //m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(center),color);
            //m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(center),color);
        }

        //btCollisionObject* colObj = static_cast<btCollisionObject*>(m_convexProxy->m_clientObject);

        if (convexBody.getCollisionShape().isConvex()) {
            tm.init(triangle[0], triangle[1], triangle[2]);
            tm.setMargin(collisionMarginTriangle);

            CollisionShape tmpShape = ob.getCollisionShape();
            ob.internalSetTemporaryCollisionShape(tm);

            CollisionAlgorithm colAlgo = ci.dispatcher1.findAlgorithm(convexBody, triBody, manifoldPtr);
            // this should use the btDispatcher, so the actual registered algorithm is used
            //		btConvexConvexAlgorithm cvxcvxalgo(m_manifoldPtr,ci,m_convexBody,m_triBody);

            resultOut.setShapeIdentifiers(-1, -1, partId, triangleIndex);
            //cvxcvxalgo.setShapeIdentifiers(-1,-1,partId,triangleIndex);
            //cvxcvxalgo.processCollision(m_convexBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
            colAlgo.processCollision(convexBody, triBody, dispatchInfoPtr, resultOut);
            //colAlgo.destroy();
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
