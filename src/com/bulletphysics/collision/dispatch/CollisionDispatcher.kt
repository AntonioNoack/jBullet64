package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.*;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.ObjectPool;

import java.util.Collections;

/**
 * CollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
 * Time of Impact, Closest Points and Penetration Depth.
 *
 * @author jezek2
 */
public class CollisionDispatcher implements Dispatcher {

    protected final ObjectPool<PersistentManifold> manifoldsPool = ObjectPool.get(PersistentManifold.class);

    private static final int MAX_BROADPHASE_COLLISION_TYPES = BroadphaseNativeType.MAX_BROADPHASE_COLLISION_TYPES.ordinal();
    private final ObjectArrayList<PersistentManifold> manifoldsPtr = new ObjectArrayList<PersistentManifold>();
    private boolean staticWarningReported = false;
    private NearCallback nearCallback;
    private final CollisionAlgorithmCreateFunc[][] doubleDispatch = new CollisionAlgorithmCreateFunc[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];
    private CollisionConfiguration collisionConfiguration;

    private final CollisionAlgorithmConstructionInfo tmpCI = new CollisionAlgorithmConstructionInfo();

    public CollisionDispatcher(CollisionConfiguration collisionConfiguration) {
        this.collisionConfiguration = collisionConfiguration;

        setNearCallback(new DefaultNearCallback());

        for (int i = 0; i < MAX_BROADPHASE_COLLISION_TYPES; i++) {
            for (int j = 0; j < MAX_BROADPHASE_COLLISION_TYPES; j++) {
                doubleDispatch[i][j] = collisionConfiguration.getCollisionAlgorithmCreateFunc(
                        BroadphaseNativeType.forValue(i),
                        BroadphaseNativeType.forValue(j)
                );
                assert (doubleDispatch[i][j] != null);
            }
        }
    }

    public void registerCollisionCreateFunc(int proxyType0, int proxyType1, CollisionAlgorithmCreateFunc createFunc) {
        doubleDispatch[proxyType0][proxyType1] = createFunc;
    }

    public NearCallback getNearCallback() {
        return nearCallback;
    }

    public void setNearCallback(NearCallback nearCallback) {
        this.nearCallback = nearCallback;
    }

    @SuppressWarnings("unused")
    public CollisionConfiguration getCollisionConfiguration() {
        return collisionConfiguration;
    }

    @SuppressWarnings("unused")
    public void setCollisionConfiguration(CollisionConfiguration collisionConfiguration) {
        this.collisionConfiguration = collisionConfiguration;
    }

    @Override
    public CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold) {
        CollisionAlgorithmConstructionInfo ci = tmpCI;
        ci.dispatcher1 = this;
        ci.manifold = sharedManifold;
        CollisionAlgorithmCreateFunc createFunc = doubleDispatch[body0.getCollisionShape().getShapeType().ordinal()][body1.getCollisionShape().getShapeType().ordinal()];
        CollisionAlgorithm algo = createFunc.createCollisionAlgorithm(ci, body0, body1);
        algo.internalSetCreateFunc(createFunc);

        return algo;
    }

    @Override
    public void freeCollisionAlgorithm(CollisionAlgorithm algo) {
        CollisionAlgorithmCreateFunc createFunc = algo.internalGetCreateFunc();
        algo.internalSetCreateFunc(null);
        createFunc.releaseCollisionAlgorithm(algo);
        algo.destroy();
    }

    @Override
    public PersistentManifold getNewManifold(Object b0, Object b1) {

        CollisionObject body0 = (CollisionObject) b0;
        CollisionObject body1 = (CollisionObject) b1;

        PersistentManifold manifold = manifoldsPool.get();
        manifold.init(body0, body1);

        manifold.index1a = manifoldsPtr.size();
        manifoldsPtr.add(manifold);

        return manifold;
    }

    @Override
    public void releaseManifold(PersistentManifold manifold) {
        clearManifold(manifold);

        int findIndex = manifold.index1a;
        assert (findIndex < manifoldsPtr.size());
        Collections.swap(manifoldsPtr, findIndex, manifoldsPtr.size() - 1);
        manifoldsPtr.getQuick(findIndex).index1a = findIndex;
        manifoldsPtr.removeQuick(manifoldsPtr.size() - 1);

        manifoldsPool.release(manifold);
    }

    @Override
    public void clearManifold(PersistentManifold manifold) {
        manifold.clearManifold();
    }

    @Override
    public boolean needsCollision(CollisionObject body0, CollisionObject body1) {
        assert (body0 != null);
        assert (body1 != null);

        boolean needsCollision = true;

        if (!staticWarningReported) {
            // broadphase filtering already deals with this
            if ((body0.isStaticObject() || body0.isKinematicObject()) &&
                    (body1.isStaticObject() || body1.isKinematicObject())) {
                staticWarningReported = true;
                System.err.println("warning CollisionDispatcher.needsCollision: static-static collision!");
            }
        }

        if (!body0.isActive() && !body1.isActive()) {
            needsCollision = false;
        } else if (!body0.checkCollideWith(body1)) {
            needsCollision = false;
        }

        return needsCollision;
    }

    @Override
    public boolean needsResponse(CollisionObject body0, CollisionObject body1) {
        //here you can do filtering
        boolean hasResponse = (body0.hasContactResponse() && body1.hasContactResponse());
        //no response between two static/kinematic bodies:
        hasResponse = hasResponse && ((!body0.isStaticOrKinematicObject()) || (!body1.isStaticOrKinematicObject()));
        return hasResponse;
    }

    private static class CollisionPairCallback implements OverlapCallback {
        private DispatcherInfo dispatchInfo;
        private CollisionDispatcher dispatcher;

        public void init(DispatcherInfo dispatchInfo, CollisionDispatcher dispatcher) {
            this.dispatchInfo = dispatchInfo;
            this.dispatcher = dispatcher;
        }

        public boolean processOverlap(BroadphasePair pair) {
            dispatcher.getNearCallback().handleCollision(pair, dispatcher, dispatchInfo);
            return false;
        }
    }

    private final CollisionPairCallback collisionPairCallback = new CollisionPairCallback();

    @Override
    public void dispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo, Dispatcher dispatcher) {
        collisionPairCallback.init(dispatchInfo, this);
        pairCache.processAllOverlappingPairs(collisionPairCallback, dispatcher);
    }

    @Override
    public int getNumManifolds() {
        return manifoldsPtr.size();
    }

    @Override
    public PersistentManifold getManifoldByIndexInternal(int index) {
        return manifoldsPtr.getQuick(index);
    }

    @Override
    public ObjectArrayList<PersistentManifold> getInternalManifoldPointer() {
        return manifoldsPtr;
    }

}
