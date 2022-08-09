/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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
package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.dispatch.CollisionAlgorithmCreateFunc;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.ManifoldResult;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.IntArrayList;
import java.util.ArrayList;

import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * Collision Algorithm for GImpact Shapes.<p>
 * <p>
 * For register this algorithm in Bullet, proceed as following:
 * <pre>
 * CollisionDispatcher dispatcher = (CollisionDispatcher)dynamicsWorld.getDispatcher();
 * GImpactCollisionAlgorithm.registerAlgorithm(dispatcher);
 * </pre>
 *
 * @author jezek2
 */
public class GImpactCollisionAlgorithm extends CollisionAlgorithm {

    protected CollisionAlgorithm convex_algorithm;
    protected PersistentManifold manifoldPtr;
    protected ManifoldResult resultOut;
    protected DispatcherInfo dispatchInfo;
    protected int triFace0, triFace1;
    protected int part0, part1;

    private final IntArrayList tmpPairSet = new IntArrayList();

    public void init(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
        super.init(ci);
        manifoldPtr = null;
        convex_algorithm = null;
    }

    @Override
    public void destroy() {
        clearCache();
    }

    @Override
    public void processCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        clearCache();

        this.resultOut = resultOut;
        this.dispatchInfo = dispatchInfo;
        GImpactShapeInterface gimpactShape0;
        GImpactShapeInterface gimpactShape1;

        if (body0.getCollisionShape().getShapeType() == BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE) {
            gimpactShape0 = (GImpactShapeInterface) body0.getCollisionShape();

            if (body1.getCollisionShape().getShapeType() == BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE) {
                gimpactShape1 = (GImpactShapeInterface) body1.getCollisionShape();
                gimpactVsGimpact(body0, body1, gimpactShape0, gimpactShape1);
            } else {
                gimpactVsShape(body0, body1, gimpactShape0, body1.getCollisionShape(), false);
            }

        } else if (body1.getCollisionShape().getShapeType() == BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE) {
            gimpactShape1 = (GImpactShapeInterface) body1.getCollisionShape();

            gimpactVsShape(body1, body0, gimpactShape1, body0.getCollisionShape(), true);
        }
    }

    public void gimpactVsGimpact(CollisionObject body0, CollisionObject body1, GImpactShapeInterface shape0, GImpactShapeInterface shape1) {
        if (shape0.getGImpactShapeType() == ShapeType.TRIMESH_SHAPE) {
            GImpactMeshShape meshshape0 = (GImpactMeshShape) shape0;
            part0 = meshshape0.getMeshPartCount();

            while ((part0--) != 0) {
                gimpactVsGimpact(body0, body1, meshshape0.getMeshPart(part0), shape1);
            }

            return;
        }

        if (shape1.getGImpactShapeType() == ShapeType.TRIMESH_SHAPE) {
            GImpactMeshShape meshshape1 = (GImpactMeshShape) shape1;
            part1 = meshshape1.getMeshPartCount();

            while ((part1--) != 0) {
                gimpactVsGimpact(body0, body1, shape0, meshshape1.getMeshPart(part1));
            }

            return;
        }

        Transform orgTrans0 = body0.getWorldTransform(new Transform());
        Transform orgTrans1 = body1.getWorldTransform(new Transform());

        IntArrayList pairSet = tmpPairSet;
        pairSet.clear();

        gimpactVsGimpactFindPairs(orgTrans0, orgTrans1, shape0, shape1, pairSet);

        if (pairSet.size() == 0) {
            return;
        }

        if (shape0.getGImpactShapeType() == ShapeType.TRIMESH_SHAPE_PART &&
                shape1.getGImpactShapeType() == ShapeType.TRIMESH_SHAPE_PART) {

            GImpactMeshShapePart shapePart0 = (GImpactMeshShapePart) shape0;
            GImpactMeshShapePart shapePart1 = (GImpactMeshShapePart) shape1;

            //specialized function
            //#ifdef BULLET_TRIANGLE_COLLISION
            //collide_gjk_triangles(body0,body1,shapepart0,shapepart1,&pairset[0].m_index1,pairset.size());
            //#else
            collideSatTriangles(body0, body1, shapePart0, shapePart1, pairSet, pairSet.size()/2);
            //#endif

            return;
        }

        // general function

        shape0.lockChildShapes();
        shape1.lockChildShapes();

        GImpactShapeRetriever retriever0 = new GImpactShapeRetriever(shape0);
        GImpactShapeRetriever retriever1 = new GImpactShapeRetriever(shape1);

        boolean childHasTransform0 = shape0.childrenHasTransform();
        boolean childHasTransform1 = shape1.childrenHasTransform();

        Transform tmpTrans = Stack.newTrans();

        int i = pairSet.size();
        while (i > 0) {
            triFace1 = pairSet.get(--i);
            triFace0 = pairSet.get(--i);
            CollisionShape colShape0 = retriever0.getChildShape(triFace0);
            CollisionShape colShape1 = retriever1.getChildShape(triFace1);

            if (childHasTransform0) {
                tmpTrans.mul(orgTrans0, shape0.getChildTransform(triFace0));
                body0.setWorldTransform(tmpTrans);
            }

            if (childHasTransform1) {
                tmpTrans.mul(orgTrans1, shape1.getChildTransform(triFace1));
                body1.setWorldTransform(tmpTrans);
            }

            // collide two convex shapes
            convexVsConvexCollision(body0, body1, colShape0, colShape1);

            if (childHasTransform0) body0.setWorldTransform(orgTrans0);
            if (childHasTransform1) body1.setWorldTransform(orgTrans1);

        }

        shape0.unlockChildShapes();
        shape1.unlockChildShapes();
    }

    public void gimpactVsShape(CollisionObject body0, CollisionObject body1, GImpactShapeInterface shape0, CollisionShape shape1, boolean swapped) {
        if (shape0.getGImpactShapeType() == ShapeType.TRIMESH_SHAPE) {
            GImpactMeshShape meshshape0 = (GImpactMeshShape) shape0;
            part0 = meshshape0.getMeshPartCount();

            while ((part0--) != 0) {
                gimpactVsShape(body0,
                        body1,
                        meshshape0.getMeshPart(part0),
                        shape1, swapped);
            }

            return;
        }

        //#ifdef GIMPACT_VS_PLANE_COLLISION
        if (shape0.getGImpactShapeType() == ShapeType.TRIMESH_SHAPE_PART &&
                shape1.getShapeType() == BroadphaseNativeType.STATIC_PLANE_PROXYTYPE) {
            GImpactMeshShapePart shapePart = (GImpactMeshShapePart) shape0;
            StaticPlaneShape planeShape = (StaticPlaneShape) shape1;
            gimpactTrimeshPartVsPlaneCollision(body0, body1, shapePart, planeShape, swapped);
            return;
        }
        //#endif

        if (shape1.isCompound()) {
            CompoundShape compoundshape = (CompoundShape) shape1;
            gimpactVsCompoundShape(body0, body1, shape0, compoundshape, swapped);
            return;
        } else if (shape1.isConcave()) {
            ConcaveShape concaveshape = (ConcaveShape) shape1;
            gimpactVsConcave(body0, body1, shape0, concaveshape, swapped);
            return;
        }

        Transform orgTrans0 = body0.getWorldTransform(new Transform());
        Transform orgTrans1 = body1.getWorldTransform(new Transform());

        IntArrayList collidedResults = new IntArrayList();

        gimpactVsShapeFindPairs(orgTrans0, orgTrans1, shape0, shape1, collidedResults);

        if (collidedResults.size() == 0) {
            return;
        }
        shape0.lockChildShapes();

        GImpactShapeRetriever retriever0 = new GImpactShapeRetriever(shape0);

        boolean childHasTransform0 = shape0.childrenHasTransform();

        Transform tmpTrans = Stack.newTrans();

        int i = collidedResults.size();

        while ((i--) != 0) {
            int child_index = collidedResults.get(i);
            if (swapped) {
                triFace1 = child_index;
            } else {
                triFace0 = child_index;
            }
            CollisionShape colShape0 = retriever0.getChildShape(child_index);

            if (childHasTransform0) {
                tmpTrans.mul(orgTrans0, shape0.getChildTransform(child_index));
                body0.setWorldTransform(tmpTrans);
            }

            // collide two shapes
            if (swapped) {
                shapeVsShapeCollision(body1, body0, shape1, colShape0);
            } else {
                shapeVsShapeCollision(body0, body1, colShape0, shape1);
            }

            // restore transforms
            if (childHasTransform0) {
                body0.setWorldTransform(orgTrans0);
            }

        }

        shape0.unlockChildShapes();
    }

    public void gimpactVsCompoundShape(CollisionObject body0, CollisionObject body1, GImpactShapeInterface shape0, CompoundShape shape1, boolean swapped) {
        Transform orgTrans1 = body1.getWorldTransform(new Transform());
        Transform childTrans1 = new Transform();
        Transform tmpTrans = Stack.newTrans();

        int i = shape1.getNumChildShapes();
        while ((i--) != 0) {
            CollisionShape colShape1 = shape1.getChildShape(i);
            childTrans1.mul(orgTrans1, shape1.getChildTransform(i, tmpTrans));

            body1.setWorldTransform(childTrans1);

            // collide child shape
            gimpactVsShape(body0, body1,
                    shape0, colShape1, swapped);

            // restore transforms
            body1.setWorldTransform(orgTrans1);
        }
    }

    public void gimpactVsConcave(CollisionObject body0, CollisionObject body1, GImpactShapeInterface shape0, ConcaveShape shape1, boolean swapped) {
        // create the callback
        GImpactTriangleCallback callback = new GImpactTriangleCallback();
        callback.algorithm = this;
        callback.body0 = body0;
        callback.body1 = body1;
        callback.gimpactShape0 = shape0;
        callback.swapped = swapped;
        callback.margin = shape1.getMargin();

        // getting the trimesh AABB
        Transform gimpactInConcaveSpace = Stack.newTrans();

        body1.getWorldTransform(gimpactInConcaveSpace);
        gimpactInConcaveSpace.inverse();
        gimpactInConcaveSpace.mul(body0.getWorldTransform(Stack.newTrans()));

        Vector3d minAABB = Stack.newVec(), maxAABB = Stack.newVec();
        shape0.getAabb(gimpactInConcaveSpace, minAABB, maxAABB);

        shape1.processAllTriangles(callback, minAABB, maxAABB);
    }

    /**
     * Creates a new contact point.
     */
    @SuppressWarnings("UnusedReturnValue")
    protected PersistentManifold newContactManifold(CollisionObject body0, CollisionObject body1) {
        manifoldPtr = dispatcher.getNewManifold(body0, body1);
        return manifoldPtr;
    }

    protected void destroyConvexAlgorithm() {
        if (convex_algorithm != null) {
            //convex_algorithm.destroy();
            dispatcher.freeCollisionAlgorithm(convex_algorithm);
            convex_algorithm = null;
        }
    }

    protected void destroyContactManifolds() {
        if (manifoldPtr == null) return;
        dispatcher.releaseManifold(manifoldPtr);
        manifoldPtr = null;
    }

    protected void clearCache() {
        destroyContactManifolds();
        destroyConvexAlgorithm();

        triFace0 = -1;
        part0 = -1;
        triFace1 = -1;
        part1 = -1;
    }

    protected PersistentManifold getLastManifold() {
        return manifoldPtr;
    }

    /**
     * Call before process collision.
     */
    protected void checkManifold(CollisionObject body0, CollisionObject body1) {
        if (getLastManifold() == null) {
            newContactManifold(body0, body1);
        }

        resultOut.setPersistentManifold(getLastManifold());
    }

    /**
     * Call before process collision.
     */
    protected CollisionAlgorithm newAlgorithm(CollisionObject body0, CollisionObject body1) {
        checkManifold(body0, body1);
        return dispatcher.findAlgorithm(body0, body1, getLastManifold());
    }

    /**
     * Call before process collision.
     */
    protected void checkConvexAlgorithm(CollisionObject body0, CollisionObject body1) {
        if (convex_algorithm != null) return;
        convex_algorithm = newAlgorithm(body0, body1);
    }

    protected void addContactPoint(CollisionObject body0, CollisionObject body1, Vector3d point, Vector3d normal, double distance) {
        resultOut.setShapeIdentifiers(part0, triFace0, part1, triFace1);
        checkManifold(body0, body1);
        resultOut.addContactPoint(normal, point, distance);
    }

	/*
	protected void collide_gjk_triangles(CollisionObject body0, CollisionObject body1, GImpactMeshShapePart shape0, GImpactMeshShapePart shape1, IntArrayList pairs, int pair_count) {
	}
	*/

    void collideSatTriangles(CollisionObject body0, CollisionObject body1, GImpactMeshShapePart shape0, GImpactMeshShapePart shape1, IntArrayList pairs, int pairCount) {
        Vector3d tmp = Stack.newVec();

        Transform orgTrans0 = body0.getWorldTransform(new Transform());
        Transform orgTrans1 = body1.getWorldTransform(new Transform());

        PrimitiveTriangle tri0 = new PrimitiveTriangle();
        PrimitiveTriangle tri1 = new PrimitiveTriangle();
        TriangleContact contactData = new TriangleContact();

        shape0.lockChildShapes();
        shape1.lockChildShapes();

        int pairPointer = 0;

        while ((pairCount--) != 0) {

            triFace0 = pairs.get(pairPointer++);
            triFace1 = pairs.get(pairPointer++);

            shape0.getPrimitiveTriangle(triFace0, tri0);
            shape1.getPrimitiveTriangle(triFace1, tri1);

            //#ifdef TRI_COLLISION_PROFILING
            //bt_begin_gim02_tri_time();
            //#endif

            tri0.applyTransform(orgTrans0);
            tri1.applyTransform(orgTrans1);

            // build planes
            tri0.buildTriPlane();
            tri1.buildTriPlane();

            // test conservative
            if (tri0.overlapTestConservative(tri1)) {
                if (tri0.findTriangleCollisionClipMethod(tri1, contactData)) {

                    int j = contactData.pointCount;
                    while ((j--) != 0) {
                        tmp.x = contactData.separatingNormal.x;
                        tmp.y = contactData.separatingNormal.y;
                        tmp.z = contactData.separatingNormal.z;

                        addContactPoint(body0, body1,
                                contactData.points[j], tmp,
                                -contactData.penetration_depth);
                    }
                }
            }

            //#ifdef TRI_COLLISION_PROFILING
            //bt_end_gim02_tri_time();
            //#endif
        }

        shape0.unlockChildShapes();
        shape1.unlockChildShapes();
    }

    protected void shapeVsShapeCollision(CollisionObject body0, CollisionObject body1, CollisionShape shape0, CollisionShape shape1) {
        CollisionShape tmpShape0 = body0.getCollisionShape();
        CollisionShape tmpShape1 = body1.getCollisionShape();

        body0.internalSetTemporaryCollisionShape(shape0);
        body1.internalSetTemporaryCollisionShape(shape1);


        CollisionAlgorithm algorithm = newAlgorithm(body0, body1);
        // post :	checkManifold is called

        resultOut.setShapeIdentifiers(part0, triFace0, part1, triFace1);

        algorithm.processCollision(body0, body1, dispatchInfo, resultOut);

        //algor.destroy();
        dispatcher.freeCollisionAlgorithm(algorithm);


        body0.internalSetTemporaryCollisionShape(tmpShape0);
        body1.internalSetTemporaryCollisionShape(tmpShape1);
    }

    protected void convexVsConvexCollision(CollisionObject body0, CollisionObject body1, CollisionShape shape0, CollisionShape shape1) {
        CollisionShape tmpShape0 = body0.getCollisionShape();
        CollisionShape tmpShape1 = body1.getCollisionShape();

        body0.internalSetTemporaryCollisionShape(shape0);
        body1.internalSetTemporaryCollisionShape(shape1);

        resultOut.setShapeIdentifiers(part0, triFace0, part1, triFace1);

        checkConvexAlgorithm(body0, body1);
        convex_algorithm.processCollision(body0, body1, dispatchInfo, resultOut);

        body0.internalSetTemporaryCollisionShape(tmpShape0);
        body1.internalSetTemporaryCollisionShape(tmpShape1);
    }

    void gimpactVsGimpactFindPairs(Transform trans0, Transform trans1, GImpactShapeInterface shape0, GImpactShapeInterface shape1, IntArrayList pairset) {
        if (shape0.hasBoxSet() && shape1.hasBoxSet()) {
            GImpactBvh.findCollision(shape0.getBoxSet(), trans0, shape1.getBoxSet(), trans1, pairset);
        } else {
            AABB boxshape0 = new AABB();
            AABB boxshape1 = new AABB();
            int i = shape0.getNumChildShapes();

            while ((i--) != 0) {
                shape0.getChildAabb(i, trans0, boxshape0.min, boxshape0.max);

                int j = shape1.getNumChildShapes();
                while ((j--) != 0) {
                    shape1.getChildAabb(i, trans1, boxshape1.min, boxshape1.max);

                    if (boxshape1.hasCollision(boxshape0)) {
                        pairset.pushPair(i, j);
                    }
                }
            }
        }
    }

    protected void gimpactVsShapeFindPairs(Transform trans0, Transform trans1, GImpactShapeInterface shape0, CollisionShape shape1, IntArrayList collided_primitives) {
        AABB boxShape = new AABB();

        if (shape0.hasBoxSet()) {
            Transform trans1to0 = new Transform();
            trans1to0.inverse(trans0);
            trans1to0.mul(trans1);

            shape1.getAabb(trans1to0, boxShape.min, boxShape.max);

            shape0.getBoxSet().boxQuery(boxShape, collided_primitives);
        } else {
            shape1.getAabb(trans1, boxShape.min, boxShape.max);

            AABB boxShape0 = new AABB();
            int i = shape0.getNumChildShapes();

            while ((i--) != 0) {
                shape0.getChildAabb(i, trans0, boxShape0.min, boxShape0.max);

                if (boxShape.hasCollision(boxShape0)) {
                    collided_primitives.add(i);
                }
            }
        }
    }

    protected void gimpactTrimeshPartVsPlaneCollision(CollisionObject body0, CollisionObject body1, GImpactMeshShapePart shape0, StaticPlaneShape shape1, boolean swapped) {
        Transform orgtrans0 = body0.getWorldTransform(new Transform());
        Transform orgtrans1 = body1.getWorldTransform(new Transform());

        Vector4d plane = new Vector4d();
        PlaneShape.getPlaneEquationTransformed(shape1, orgtrans1, plane);

        // test box against plane

        AABB tribox = new AABB();
        shape0.getAabb(orgtrans0, tribox.min, tribox.max);
        tribox.incrementMargin(shape1.getMargin());

        if (tribox.plane_classify(plane) != PlaneIntersectionType.COLLIDE_PLANE) {
            return;
        }
        shape0.lockChildShapes();

        double margin = shape0.getMargin() + shape1.getMargin();

        Vector3d vertex = Stack.newVec();

        Vector3d tmp = Stack.newVec();

        int vi = shape0.getVertexCount();
        while ((vi--) != 0) {
            shape0.getVertex(vi, vertex);
            orgtrans0.transform(vertex);

            double distance = VectorUtil.dot3(vertex, plane) - plane.w - margin;

            if (distance < 0.0)//add contact
            {
                if (swapped) {
                    tmp.set(-plane.x, -plane.y, -plane.z);
                    addContactPoint(body1, body0, vertex, tmp, distance);
                } else {
                    tmp.set(plane.x, plane.y, plane.z);
                    addContactPoint(body0, body1, vertex, tmp, distance);
                }
            }
        }

        shape0.unlockChildShapes();
    }


    public void setFace0(int value) {
        triFace0 = value;
    }

    public int getFace0() {
        return triFace0;
    }

    public void setFace1(int value) {
        triFace1 = value;
    }

    public int getFace1() {
        return triFace1;
    }

    public void setPart0(int value) {
        part0 = value;
    }

    public int getPart0() {
        return part0;
    }

    public void setPart1(int value) {
        part1 = value;
    }

    public int getPart1() {
        return part1;
    }

    @Override
    public double calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
        return 1.0;
    }

    @Override
    public void getAllContactManifolds(ArrayList<PersistentManifold> manifoldArray) {
        if (manifoldPtr != null) {
            manifoldArray.add(manifoldPtr);
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    /**
     * Use this function for register the algorithm externally.
     */
    public static void registerAlgorithm(CollisionDispatcher dispatcher) {
        CreateFunc createFunc = new CreateFunc();

        for (int i = 0; i < BroadphaseNativeType.MAX_BROADPHASE_COLLISION_TYPES.ordinal(); i++) {
            dispatcher.registerCollisionCreateFunc(BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE.ordinal(), i, createFunc);
        }

        for (int i = 0; i < BroadphaseNativeType.MAX_BROADPHASE_COLLISION_TYPES.ordinal(); i++) {
            dispatcher.registerCollisionCreateFunc(i, BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE.ordinal(), createFunc);
        }
    }

    public static class CreateFunc extends CollisionAlgorithmCreateFunc {
        private final ObjectPool<GImpactCollisionAlgorithm> pool = ObjectPool.get(GImpactCollisionAlgorithm.class);

        @Override
        public CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
            GImpactCollisionAlgorithm algo = pool.get();
            algo.init(ci, body0, body1);
            return algo;
        }

        @Override
        public void releaseCollisionAlgorithm(CollisionAlgorithm algo) {
            pool.release((GImpactCollisionAlgorithm) algo);
        }
    }

}
