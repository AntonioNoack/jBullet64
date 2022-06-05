package com.bulletphysics.collision.dispatch;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.broadphase.*;
import com.bulletphysics.collision.narrowphase.*;
import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.shapes.*;
import com.bulletphysics.linearmath.*;
import java.util.ArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * CollisionWorld is interface and container for the collision detection.
 *
 * @author jezek2
 */
public class CollisionWorld {

    //protected final BulletStack stack = BulletStack.get();

    protected ArrayList<CollisionObject> collisionObjects = new ArrayList<>();
    protected Dispatcher dispatcher1;
    protected DispatcherInfo dispatchInfo = new DispatcherInfo();
    //protected btStackAlloc*	m_stackAlloc;
    protected BroadphaseInterface broadphasePairCache;
    protected IDebugDraw debugDrawer;

    /**
     * This constructor doesn't own the dispatcher and paircache/broadphase.
     */
    public CollisionWorld(Dispatcher dispatcher, BroadphaseInterface broadphasePairCache, CollisionConfiguration collisionConfiguration) {
        this.dispatcher1 = dispatcher;
        this.broadphasePairCache = broadphasePairCache;
    }

    public void destroy() {
        // clean up remaining objects
        for (CollisionObject collisionObject : collisionObjects) {
            BroadphaseProxy bp = collisionObject.getBroadphaseHandle();
            if (bp != null) {
                //
                // only clear the cached algorithms
                //
                getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(bp, dispatcher1);
                getBroadphase().destroyProxy(bp, dispatcher1);
            }
        }
    }

    public void addCollisionObject(CollisionObject collisionObject) {
        addCollisionObject(collisionObject, CollisionFilterGroups.DEFAULT_FILTER, CollisionFilterGroups.ALL_FILTER);
    }

    public void addCollisionObject(CollisionObject collisionObject, short collisionFilterGroup, short collisionFilterMask) {
        // check that the object isn't already added
        assert (!collisionObjects.contains(collisionObject));

        collisionObjects.add(collisionObject);

        // calculate new AABB
        // TODO: check if it's overwritten or not
        Transform trans = collisionObject.getWorldTransform(Stack.newTrans());

        Vector3d minAabb = Stack.newVec();
        Vector3d maxAabb = Stack.newVec();
        collisionObject.getCollisionShape().getAabb(trans, minAabb, maxAabb);

        BroadphaseNativeType type = collisionObject.getCollisionShape().getShapeType();
        collisionObject.setBroadphaseHandle(getBroadphase().createProxy(
                minAabb,
                maxAabb,
                type,
                collisionObject,
                collisionFilterGroup,
                collisionFilterMask,
                dispatcher1, null));
    }

    public void performDiscreteCollisionDetection() {

        updateAabbs();

        broadphasePairCache.calculateOverlappingPairs(dispatcher1);

        Dispatcher dispatcher = getDispatcher();
        if (dispatcher != null) {
            dispatcher.dispatchAllCollisionPairs(broadphasePairCache.getOverlappingPairCache(), dispatchInfo, dispatcher1);
        }

    }

    public void removeCollisionObject(CollisionObject collisionObject) {

        BroadphaseProxy bp = collisionObject.getBroadphaseHandle();
        if (bp != null) {
            //
            // only clear the cached algorithms
            //
            getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(bp, dispatcher1);
            getBroadphase().destroyProxy(bp, dispatcher1);
            collisionObject.setBroadphaseHandle(null);
        }

        collisionObjects.remove(collisionObject);
    }

    public void setBroadphase(BroadphaseInterface pairCache) {
        broadphasePairCache = pairCache;
    }

    public BroadphaseInterface getBroadphase() {
        return broadphasePairCache;
    }

    public OverlappingPairCache getPairCache() {
        return broadphasePairCache.getOverlappingPairCache();
    }

    public Dispatcher getDispatcher() {
        return dispatcher1;
    }

    public DispatcherInfo getDispatchInfo() {
        return dispatchInfo;
    }

    private static boolean updateAabbs_reportMe = true;

    // JAVA NOTE: ported from 2.74, missing contact threshold stuff
    public void updateSingleAabb(CollisionObject colObj) {

        Vector3d minAabb = Stack.newVec(), maxAabb = Stack.newVec();
        Vector3d tmp = Stack.newVec();
        Transform tmpTrans = Stack.newTrans();

        colObj.getCollisionShape().getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
        // need to increase the aabb for contact thresholds
        Vector3d contactThreshold = Stack.newVec();
        contactThreshold.set(BulletGlobals.getContactBreakingThreshold(), BulletGlobals.getContactBreakingThreshold(), BulletGlobals.getContactBreakingThreshold());
        minAabb.sub(contactThreshold);
        maxAabb.add(contactThreshold);

        BroadphaseInterface bp = broadphasePairCache;

        // moving objects should be moderately sized, probably something wrong if not
        tmp.sub(maxAabb, minAabb); // TODO: optimize
        if (colObj.isStaticObject() || (tmp.lengthSquared() < 1e12f)) {
            bp.setAabb(colObj.getBroadphaseHandle(), minAabb, maxAabb, dispatcher1);
        } else {
            // something went wrong, investigate
            // this assert is unwanted in 3D modelers (danger of loosing work)
            colObj.setActivationState(CollisionObject.DISABLE_SIMULATION);

            if (updateAabbs_reportMe && debugDrawer != null) {
                updateAabbs_reportMe = false;
                debugDrawer.reportErrorWarning("Overflow in AABB, object removed from simulation");
                debugDrawer.reportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n");
                debugDrawer.reportErrorWarning("Please include above information, your Platform, version of OS.\n");
                debugDrawer.reportErrorWarning("Thanks.\n");
            }
        }

        Stack.subVec(4);
        Stack.subTrans(1);

    }

    public void updateAabbs() {
        for (CollisionObject colObj : collisionObjects) {
            // only update aabb of active objects
            if (colObj.isActive()) {
                updateSingleAabb(colObj);
            }
        }
    }

    public IDebugDraw getDebugDrawer() {
        return debugDrawer;
    }

    public void setDebugDrawer(IDebugDraw debugDrawer) {
        this.debugDrawer = debugDrawer;
    }

    public int getNumCollisionObjects() {
        return collisionObjects.size();
    }

    public static void rayTestSingle(Transform rayFromTrans, Transform rayToTrans,
                                     CollisionObject collisionObject,
                                     CollisionShape collisionShape,
                                     Transform colObjWorldTransform,
                                     RayResultCallback resultCallback) {

        if (collisionShape.isConvex()) {

            SphereShape pointShape = new SphereShape(0f);
            pointShape.setMargin(0f);

            CastResult castResult = new CastResult();
            castResult.fraction = resultCallback.closestHitFraction;

            ConvexShape convexShape = (ConvexShape) collisionShape;
            VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();

            //#define USE_SUBSIMPLEX_CONVEX_CAST 1
            //#ifdef USE_SUBSIMPLEX_CONVEX_CAST
            SubSimplexConvexCast convexCaster = new SubSimplexConvexCast(pointShape, convexShape, simplexSolver);
            //#else
            //btGjkConvexCast	convexCaster(castShape,convexShape,&simplexSolver);
            //btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);
            //#endif //#USE_SUBSIMPLEX_CONVEX_CAST

            if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, colObjWorldTransform, colObjWorldTransform, castResult)) {
                //add hit
                if (castResult.normal.lengthSquared() > 0.0001) {
                    if (castResult.fraction < resultCallback.closestHitFraction) {
                        //#ifdef USE_SUBSIMPLEX_CONVEX_CAST
                        //rotate normal into worldspace
                        rayFromTrans.basis.transform(castResult.normal);
                        //#endif //USE_SUBSIMPLEX_CONVEX_CAST

                        castResult.normal.normalize();
                        LocalRayResult localRayResult = new LocalRayResult(
                                collisionObject,
                                null,
                                castResult.normal,
                                castResult.fraction
                        );

                        boolean normalInWorldSpace = true;
                        resultCallback.addSingleResult(localRayResult, normalInWorldSpace);
                    }
                }
            }
        } else {
            if (collisionShape.isConcave()) {
                if (collisionShape.getShapeType() == BroadphaseNativeType.TRIANGLE_MESH_SHAPE_PROXYTYPE) {
                    // optimized version for BvhTriangleMeshShape
                    BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape) collisionShape;
                    Transform worldToCollisionObject = Stack.newTrans();
                    worldToCollisionObject.inverse(colObjWorldTransform);
                    Vector3d rayFromLocal = Stack.newVec(rayFromTrans.origin);
                    worldToCollisionObject.transform(rayFromLocal);
                    Vector3d rayToLocal = Stack.newVec(rayToTrans.origin);
                    worldToCollisionObject.transform(rayToLocal);

                    BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback, collisionObject, triangleMesh);
                    rcb.hitFraction = resultCallback.closestHitFraction;
                    triangleMesh.performRaycast(rcb, rayFromLocal, rayToLocal);
                } else {
                    ConcaveShape triangleMesh = (ConcaveShape) collisionShape;

                    Transform worldToCollisionObject = Stack.newTrans();
                    worldToCollisionObject.inverse(colObjWorldTransform);

                    Vector3d rayFromLocal = Stack.newVec(rayFromTrans.origin);
                    worldToCollisionObject.transform(rayFromLocal);
                    Vector3d rayToLocal = Stack.newVec(rayToTrans.origin);
                    worldToCollisionObject.transform(rayToLocal);

                    BridgeTriangleRaycastCallback rcb = new BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback, collisionObject, triangleMesh);
                    rcb.hitFraction = resultCallback.closestHitFraction;

                    Vector3d rayAabbMinLocal = Stack.newVec(rayFromLocal);
                    VectorUtil.setMin(rayAabbMinLocal, rayToLocal);
                    Vector3d rayAabbMaxLocal = Stack.newVec(rayFromLocal);
                    VectorUtil.setMax(rayAabbMaxLocal, rayToLocal);

                    triangleMesh.processAllTriangles(rcb, rayAabbMinLocal, rayAabbMaxLocal);
                }
            } else {
                // todo: use AABB tree or other BVH acceleration structure!
                if (collisionShape.isCompound()) {
                    CompoundShape compoundShape = (CompoundShape) collisionShape;
                    Transform childTrans = Stack.newTrans();
                    for (int i = 0; i < compoundShape.getNumChildShapes(); i++) {
                        compoundShape.getChildTransform(i, childTrans);
                        CollisionShape childCollisionShape = compoundShape.getChildShape(i);
                        Transform childWorldTrans = Stack.newTrans(colObjWorldTransform);
                        childWorldTrans.mul(childTrans);
                        // replace collision shape so that callback can determine the triangle
                        CollisionShape saveCollisionShape = collisionObject.getCollisionShape();
                        collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
                        rayTestSingle(rayFromTrans, rayToTrans,
                                collisionObject,
                                childCollisionShape,
                                childWorldTrans,
                                resultCallback);
                        // restore
                        collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
                    }
                }
            }
        }
    }

    private static class BridgeTriangleConvexcastCallback extends TriangleConvexcastCallback {
        public ConvexResultCallback resultCallback;
        public CollisionObject collisionObject;
        public TriangleMeshShape triangleMesh;
        public boolean normalInWorldSpace;

        public BridgeTriangleConvexcastCallback(ConvexShape castShape, Transform from, Transform to, ConvexResultCallback resultCallback, CollisionObject collisionObject, TriangleMeshShape triangleMesh, Transform triangleToWorld) {
            super(castShape, from, to, triangleToWorld, triangleMesh.getMargin());
            this.resultCallback = resultCallback;
            this.collisionObject = collisionObject;
            this.triangleMesh = triangleMesh;
        }

        @Override
        public double reportHit(Vector3d hitNormalLocal, Vector3d hitPointLocal, double hitFraction, int partId, int triangleIndex) {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.shapePart = partId;
            shapeInfo.triangleIndex = triangleIndex;
            if (hitFraction <= resultCallback.closestHitFraction) {
                LocalConvexResult convexResult = new LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal, hitPointLocal, hitFraction);
                return resultCallback.addSingleResult(convexResult, normalInWorldSpace);
            }
            return hitFraction;
        }
    }

    /**
     * objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
     */
    public static void objectQuerySingle(ConvexShape castShape, Transform convexFromTrans, Transform convexToTrans, CollisionObject collisionObject, CollisionShape collisionShape, Transform colObjWorldTransform, ConvexResultCallback resultCallback, double allowedPenetration) {
        if (collisionShape.isConvex()) {
            CastResult castResult = new CastResult();
            castResult.allowedPenetration = allowedPenetration;
            castResult.fraction = 1.0; // ??

            ConvexShape convexShape = (ConvexShape) collisionShape;
            VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
            GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new GjkEpaPenetrationDepthSolver();

            // JAVA TODO: should be convexCaster1
            //ContinuousConvexCollision convexCaster1(castShape,convexShape,&simplexSolver,&gjkEpaPenetrationSolver);
            GjkConvexCast convexCaster2 = new GjkConvexCast(castShape, convexShape, simplexSolver);
            //btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

            ConvexCast castPtr = convexCaster2;

            if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult)) {
                // add hit
                if (castResult.normal.lengthSquared() > 0.0001) {
                    if (castResult.fraction < resultCallback.closestHitFraction) {
                        castResult.normal.normalize();
                        LocalConvexResult localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal, castResult.hitPoint, castResult.fraction);

                        boolean normalInWorldSpace = true;
                        resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
                    }
                }
            }
        } else {
            if (collisionShape.isConcave()) {
                if (collisionShape.getShapeType() == BroadphaseNativeType.TRIANGLE_MESH_SHAPE_PROXYTYPE) {
                    BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape) collisionShape;
                    Transform worldTocollisionObject = Stack.newTrans();
                    worldTocollisionObject.inverse(colObjWorldTransform);

                    Vector3d convexFromLocal = Stack.newVec();
                    convexFromLocal.set(convexFromTrans.origin);
                    worldTocollisionObject.transform(convexFromLocal);

                    Vector3d convexToLocal = Stack.newVec();
                    convexToLocal.set(convexToTrans.origin);
                    worldTocollisionObject.transform(convexToLocal);

                    // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
                    Transform rotationXform = Stack.newTrans();
                    Matrix3d tmpMat = Stack.newMat();
                    tmpMat.mul(worldTocollisionObject.basis, convexToTrans.basis);
                    rotationXform.set(tmpMat);

                    BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans, resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
                    tccb.hitFraction = resultCallback.closestHitFraction;
                    tccb.normalInWorldSpace = true;

                    Vector3d boxMinLocal = Stack.newVec();
                    Vector3d boxMaxLocal = Stack.newVec();
                    castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
                    triangleMesh.performConvexcast(tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal);
                } else {
                    BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape) collisionShape;
                    Transform worldTocollisionObject = Stack.newTrans();
                    worldTocollisionObject.inverse(colObjWorldTransform);

                    Vector3d convexFromLocal = Stack.newVec();
                    convexFromLocal.set(convexFromTrans.origin);
                    worldTocollisionObject.transform(convexFromLocal);

                    Vector3d convexToLocal = Stack.newVec();
                    convexToLocal.set(convexToTrans.origin);
                    worldTocollisionObject.transform(convexToLocal);

                    // rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
                    Transform rotationXform = Stack.newTrans();
                    Matrix3d tmpMat = Stack.newMat();
                    tmpMat.mul(worldTocollisionObject.basis, convexToTrans.basis);
                    rotationXform.set(tmpMat);

                    BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans, resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
                    tccb.hitFraction = resultCallback.closestHitFraction;
                    tccb.normalInWorldSpace = false;
                    Vector3d boxMinLocal = Stack.newVec();
                    Vector3d boxMaxLocal = Stack.newVec();
                    castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);

                    Vector3d rayAabbMinLocal = Stack.newVec(convexFromLocal);
                    VectorUtil.setMin(rayAabbMinLocal, convexToLocal);
                    Vector3d rayAabbMaxLocal = Stack.newVec(convexFromLocal);
                    VectorUtil.setMax(rayAabbMaxLocal, convexToLocal);
                    rayAabbMinLocal.add(boxMinLocal);
                    rayAabbMaxLocal.add(boxMaxLocal);
                    triangleMesh.processAllTriangles(tccb, rayAabbMinLocal, rayAabbMaxLocal);
                }
            } else {
                // todo: use AABB tree or other BVH acceleration structure!
                if (collisionShape.isCompound()) {
                    CompoundShape compoundShape = (CompoundShape) collisionShape;
                    for (int i = 0; i < compoundShape.getNumChildShapes(); i++) {
                        Transform childTrans = compoundShape.getChildTransform(i, Stack.newTrans());
                        CollisionShape childCollisionShape = compoundShape.getChildShape(i);
                        Transform childWorldTrans = Stack.newTrans();
                        childWorldTrans.mul(colObjWorldTransform, childTrans);
                        // replace collision shape so that callback can determine the triangle
                        CollisionShape saveCollisionShape = collisionObject.getCollisionShape();
                        collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
                        objectQuerySingle(castShape, convexFromTrans, convexToTrans,
                                collisionObject,
                                childCollisionShape,
                                childWorldTrans,
                                resultCallback, allowedPenetration);
                        // restore
                        collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
                    }
                }
            }
        }
    }

    /**
     * rayTest performs a raycast on all objects in the CollisionWorld, and calls the resultCallback.
     * This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
     */
    public void rayTest(Vector3d rayFromWorld, Vector3d rayToWorld, RayResultCallback resultCallback) {
        Transform rayFromTrans = Stack.newTrans(), rayToTrans = Stack.newTrans();
        rayFromTrans.setIdentity();
        rayFromTrans.origin.set(rayFromWorld);
        rayToTrans.setIdentity();

        rayToTrans.origin.set(rayToWorld);

        // go over all objects, and if the ray intersects their aabb, do a ray-shape query using convexCaster (CCD)
        Vector3d collisionObjectAabbMin = Stack.newVec(), collisionObjectAabbMax = Stack.newVec();
        double[] hitLambda = new double[1];

        Transform tmpTrans = Stack.newTrans();
        Vector3d hitNormal = Stack.newVec();
        for (CollisionObject object : collisionObjects) {
            // terminate further ray tests, once the closestHitFraction reached zero
            if (resultCallback.closestHitFraction == 0.0) {
                break;
            }

            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(object.getBroadphaseHandle())) {
                //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
                object.getCollisionShape().getAabb(object.getWorldTransform(tmpTrans), collisionObjectAabbMin, collisionObjectAabbMax);

                hitLambda[0] = resultCallback.closestHitFraction;
                if (AabbUtil2.rayAabb(rayFromWorld, rayToWorld, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
                    rayTestSingle(rayFromTrans, rayToTrans,
                            object,
                            object.getCollisionShape(),
                            object.getWorldTransform(tmpTrans),
                            resultCallback);
                }
            }

        }

        Stack.subTrans(3);
        Stack.subVec(3);

    }

    /**
     * convexTest performs a swept convex cast on all objects in the {@link CollisionWorld}, and calls the resultCallback
     * This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
     */
    public void convexSweepTest(ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld, ConvexResultCallback resultCallback) {

        int v3 = Stack.getVecPosition();

        Transform convexFromTrans = Stack.newTrans();
        Transform convexToTrans = Stack.newTrans();

        convexFromTrans.set(convexFromWorld);
        convexToTrans.set(convexToWorld);

        Vector3d castShapeAabbMin = Stack.newVec();
        Vector3d castShapeAabbMax = Stack.newVec();

        // Compute AABB that encompasses angular movement
        {
            Vector3d linVel = Stack.newVec();
            Vector3d angVel = Stack.newVec();
            TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1.0, linVel, angVel);
            Transform R = Stack.newTrans();
            R.setIdentity();
            R.setRotation(convexFromTrans.getRotation(Stack.newQuat()));
            castShape.calculateTemporalAabb(R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
        }

        Transform tmpTrans = Stack.newTrans();
        Vector3d collisionObjectAabbMin = Stack.newVec();
        Vector3d collisionObjectAabbMax = Stack.newVec();
        double[] hitLambda = new double[1];

        // go over all objects, and if the ray intersects their aabb + cast shape aabb,
        // do a ray-shape query using convexCaster (CCD)
        Vector3d hitNormal = Stack.newVec();
        for (CollisionObject collisionObject : collisionObjects) {
            // only perform raycast if filterMask matches
            if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle())) {
                //RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
                collisionObject.getWorldTransform(tmpTrans);
                collisionObject.getCollisionShape().getAabb(tmpTrans, collisionObjectAabbMin, collisionObjectAabbMax);
                AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
                hitLambda[0] = 1.0; // could use resultCallback.closestHitFraction, but needs testing
                if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
                    objectQuerySingle(castShape, convexFromTrans, convexToTrans,
                            collisionObject,
                            collisionObject.getCollisionShape(),
                            tmpTrans,
                            resultCallback,
                            getDispatchInfo().allowedCcdPenetration);
                }
            }
        }

        Stack.resetVec(v3);
        Stack.subTrans(4);

    }

    public ArrayList<CollisionObject> getCollisionObjectArray() {
        return collisionObjects;
    }

    ////////////////////////////////////////////////////////////////////////////

    /**
     * LocalShapeInfo gives extra information for complex shapes.
     * Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart.
     */
    public static class LocalShapeInfo {
        public int shapePart;
        public int triangleIndex;
        //const btCollisionShape*	m_shapeTemp;
        //const btTransform*	m_shapeLocalTransform;
    }

    public static class LocalRayResult {
        public CollisionObject collisionObject;
        public LocalShapeInfo localShapeInfo;
        public final Vector3d hitNormalLocal = new Vector3d();
        public double hitFraction;

        public LocalRayResult(CollisionObject collisionObject, LocalShapeInfo localShapeInfo, Vector3d hitNormalLocal, double hitFraction) {
            this.collisionObject = collisionObject;
            this.localShapeInfo = localShapeInfo;
            this.hitNormalLocal.set(hitNormalLocal);
            this.hitFraction = hitFraction;
        }
    }

    /**
     * RayResultCallback is used to report new raycast results.
     */
    public static abstract class RayResultCallback {
        public double closestHitFraction = 1.0;
        public CollisionObject collisionObject;
        public short collisionFilterGroup = CollisionFilterGroups.DEFAULT_FILTER;
        public short collisionFilterMask = CollisionFilterGroups.ALL_FILTER;

        public boolean hasHit() {
            return (collisionObject != null);
        }

        public boolean needsCollision(BroadphaseProxy proxy0) {
            boolean collides = ((proxy0.collisionFilterGroup & collisionFilterMask) & 0xFFFF) != 0;
            collides = collides && ((collisionFilterGroup & proxy0.collisionFilterMask) & 0xFFFF) != 0;
            return collides;
        }

        public abstract double addSingleResult(LocalRayResult rayResult, boolean normalInWorldSpace);
    }

    public static class ClosestRayResultCallback extends RayResultCallback {
        public final Vector3d rayFromWorld = new Vector3d(); //used to calculate hitPointWorld from hitFraction
        public final Vector3d rayToWorld = new Vector3d();

        public final Vector3d hitNormalWorld = new Vector3d();
        public final Vector3d hitPointWorld = new Vector3d();

        public ClosestRayResultCallback(Vector3d rayFromWorld, Vector3d rayToWorld) {
            this.rayFromWorld.set(rayFromWorld);
            this.rayToWorld.set(rayToWorld);
        }

        @Override
        public double addSingleResult(LocalRayResult rayResult, boolean normalInWorldSpace) {
            // caller already does the filter on the closestHitFraction
            assert (rayResult.hitFraction <= closestHitFraction);

            closestHitFraction = rayResult.hitFraction;
            collisionObject = rayResult.collisionObject;
            if (normalInWorldSpace) {
                hitNormalWorld.set(rayResult.hitNormalLocal);
            } else {
                // need to transform normal into worldspace
                hitNormalWorld.set(rayResult.hitNormalLocal);
                collisionObject.getWorldTransform(Stack.newTrans()).basis.transform(hitNormalWorld);
            }

            VectorUtil.setInterpolate3(hitPointWorld, rayFromWorld, rayToWorld, rayResult.hitFraction);
            return rayResult.hitFraction;
        }
    }

    public static class LocalConvexResult {
        public CollisionObject hitCollisionObject;
        public LocalShapeInfo localShapeInfo;
        public final Vector3d hitNormalLocal = new Vector3d();
        public final Vector3d hitPointLocal = new Vector3d();
        public double hitFraction;

        public LocalConvexResult(CollisionObject hitCollisionObject, LocalShapeInfo localShapeInfo, Vector3d hitNormalLocal, Vector3d hitPointLocal, double hitFraction) {
            this.hitCollisionObject = hitCollisionObject;
            this.localShapeInfo = localShapeInfo;
            this.hitNormalLocal.set(hitNormalLocal);
            this.hitPointLocal.set(hitPointLocal);
            this.hitFraction = hitFraction;
        }
    }

    public static abstract class ConvexResultCallback {
        public double closestHitFraction = 1.0;
        public short collisionFilterGroup = CollisionFilterGroups.DEFAULT_FILTER;
        public short collisionFilterMask = CollisionFilterGroups.ALL_FILTER;

        public boolean hasHit() {
            return (closestHitFraction < 1.0);
        }

        public boolean needsCollision(BroadphaseProxy proxy0) {
            boolean collides = ((proxy0.collisionFilterGroup & collisionFilterMask) & 0xFFFF) != 0;
            collides = collides && ((collisionFilterGroup & proxy0.collisionFilterMask) & 0xFFFF) != 0;
            return collides;
        }

        public abstract double addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace);
    }

    public static class ClosestConvexResultCallback extends ConvexResultCallback {
        public final Vector3d convexFromWorld = new Vector3d(); // used to calculate hitPointWorld from hitFraction
        public final Vector3d convexToWorld = new Vector3d();
        public final Vector3d hitNormalWorld = new Vector3d();
        public final Vector3d hitPointWorld = new Vector3d();
        public CollisionObject hitCollisionObject;

        public ClosestConvexResultCallback(Vector3d convexFromWorld, Vector3d convexToWorld) {
            this.convexFromWorld.set(convexFromWorld);
            this.convexToWorld.set(convexToWorld);
            this.hitCollisionObject = null;
        }

        @Override
        public double addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {
            // caller already does the filter on the m_closestHitFraction
            assert (convexResult.hitFraction <= closestHitFraction);

            closestHitFraction = convexResult.hitFraction;
            hitCollisionObject = convexResult.hitCollisionObject;
            if (normalInWorldSpace) {
                hitNormalWorld.set(convexResult.hitNormalLocal);
                if (hitNormalWorld.length() > 2) {
                    System.out.println("CollisionWorld.addSingleResult world " + hitNormalWorld);
                }
            } else {
                // need to transform normal into worldspace
                hitNormalWorld.set(convexResult.hitNormalLocal);
                hitCollisionObject.getWorldTransform(Stack.newTrans()).basis.transform(hitNormalWorld);
                if (hitNormalWorld.length() > 2) {
                    System.out.println("CollisionWorld.addSingleResult world " + hitNormalWorld);
                }
            }

            hitPointWorld.set(convexResult.hitPointLocal);
            return convexResult.hitFraction;
        }
    }

    private static class BridgeTriangleRaycastCallback extends TriangleRaycastCallback {
        public RayResultCallback resultCallback;
        public CollisionObject collisionObject;
        public ConcaveShape triangleMesh;

        public BridgeTriangleRaycastCallback(Vector3d from, Vector3d to, RayResultCallback resultCallback, CollisionObject collisionObject, ConcaveShape triangleMesh) {
            super(from, to);
            this.resultCallback = resultCallback;
            this.collisionObject = collisionObject;
            this.triangleMesh = triangleMesh;
        }

        public double reportHit(Vector3d hitNormalLocal, double hitFraction, int partId, int triangleIndex) {
            LocalShapeInfo shapeInfo = new LocalShapeInfo();
            shapeInfo.shapePart = partId;
            shapeInfo.triangleIndex = triangleIndex;

            LocalRayResult rayResult = new LocalRayResult(collisionObject, shapeInfo, hitNormalLocal, hitFraction);
            return resultCallback.addSingleResult(rayResult, false);
        }
    }

}
