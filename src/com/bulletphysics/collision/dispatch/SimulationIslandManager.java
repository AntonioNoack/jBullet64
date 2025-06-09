package com.bulletphysics.collision.dispatch;

import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import java.util.Comparator;

/**
 * SimulationIslandManager creates and handles simulation islands, using {@link UnionFind}.
 *
 * @author jezek2
 */
public class SimulationIslandManager {

    private final UnionFind unionFind = new UnionFind();

    private final ObjectArrayList<PersistentManifold> islandManifold = new ObjectArrayList<PersistentManifold>();
    private final ObjectArrayList<CollisionObject> islandBodies = new ObjectArrayList<CollisionObject>();

    public void initUnionFind(int n) {
        unionFind.reset(n);
    }

    public UnionFind getUnionFind() {
        return unionFind;
    }

    public void findUnions(Dispatcher dispatcher, CollisionWorld colWorld) {
        ObjectArrayList<BroadphasePair> pairPtr = colWorld.getPairCache().getOverlappingPairArray();
        for (int i = 0; i < pairPtr.size(); i++) {
            BroadphasePair collisionPair = pairPtr.getQuick(i);

            CollisionObject colObj0 = (CollisionObject) collisionPair.proxy0.clientObject;
            CollisionObject colObj1 = (CollisionObject) collisionPair.proxy1.clientObject;

            if (((colObj0 != null) && ((colObj0).mergesSimulationIslands())) &&
                    ((colObj1 != null) && ((colObj1).mergesSimulationIslands()))) {
                unionFind.combineIslands((colObj0).getIslandTag(), (colObj1).getIslandTag());
            }
        }
    }

    public void updateActivationState(CollisionWorld colWorld, Dispatcher dispatcher) {
        initUnionFind(colWorld.getCollisionObjectArray().size());

        // put the index into m_controllers into m_tag
        {
            int index = 0;
            int i;
            for (i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
                CollisionObject collisionObject = colWorld.getCollisionObjectArray().getQuick(i);
                collisionObject.setIslandTag(index);
                collisionObject.setCompanionId(-1);
                collisionObject.setHitFraction(1.0);
                index++;
            }
        }
        // do the union find

        findUnions(dispatcher, colWorld);
    }

    public void storeIslandActivationState(CollisionWorld colWorld) {
        // put the islandId ('find' value) into m_tag
        for (int i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
            CollisionObject collisionObject = colWorld.getCollisionObjectArray().getQuick(i);
            if (!collisionObject.isStaticOrKinematicObject()) {
                collisionObject.setIslandTag(unionFind.findGroupId(i));
                collisionObject.setCompanionId(-1);
            } else {
                collisionObject.setIslandTag(-1);
                collisionObject.setCompanionId(-2);
            }
        }
    }

    private static int getIslandId(PersistentManifold lhs) {
        int islandId;
        CollisionObject rcolObj0 = (CollisionObject) lhs.getBody0();
        CollisionObject rcolObj1 = (CollisionObject) lhs.getBody1();
        islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
        return islandId;
    }

    public void buildIslands(Dispatcher dispatcher, ObjectArrayList<CollisionObject> collisionObjects) {
        BulletStats.pushProfile("islandUnionFindAndQuickSort");
        try {
            islandManifold.clear();

            // we are going to sort the unionfind array, and store the element id in the size
            // afterward, we clean unionfind, to make sure no-one uses it anymore

            getUnionFind().sortIslands();
            int numElem = getUnionFind().getNumElements();

            int endIslandIndex;
            int startIslandIndex;

            // update the sleeping state for bodies, if all are sleeping
            for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
                int islandId = getUnionFind().getParent(startIslandIndex);

                endIslandIndex = startIslandIndex + 1;
                while ((endIslandIndex < numElem) && (getUnionFind().getParent(endIslandIndex) == islandId)) {
                    endIslandIndex++;
                }

                boolean allSleeping = true;

                int idx;
                for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                    int sz = getUnionFind().getRank(idx);

                    CollisionObject colObj0 = collisionObjects.getQuick(sz);

                    assert ((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));
                    if (colObj0.getIslandTag() == islandId) {
                        if (colObj0.getActivationState() == CollisionObject.ACTIVE_TAG) {
                            allSleeping = false;
                        }
                        if (colObj0.getActivationState() == CollisionObject.DISABLE_DEACTIVATION) {
                            allSleeping = false;
                        }
                    }
                }


                if (allSleeping) {
                    //int idx;
                    for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                        int sz = getUnionFind().getRank(idx);
                        CollisionObject colObj0 = collisionObjects.getQuick(sz);

                        assert ((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));

                        if (colObj0.getIslandTag() == islandId) {
                            colObj0.setActivationState(CollisionObject.ISLAND_SLEEPING);
                        }
                    }
                } else {

                    //int idx;
                    for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                        int i = getUnionFind().getRank(idx);

                        CollisionObject colObj0 = collisionObjects.getQuick(i);

                        assert ((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));
                        if (colObj0.getIslandTag() == islandId) {
                            if (colObj0.getActivationState() == CollisionObject.ISLAND_SLEEPING) {
                                colObj0.setActivationState(CollisionObject.WANTS_DEACTIVATION);
                            }
                        }
                    }
                }
            }


            int i;
            int maxNumManifolds = dispatcher.getNumManifolds();

            //#define SPLIT_ISLANDS 1
            //#ifdef SPLIT_ISLANDS
            //#endif //SPLIT_ISLANDS

            for (i = 0; i < maxNumManifolds; i++) {
                PersistentManifold manifold = dispatcher.getManifoldByIndexInternal(i);

                CollisionObject colObj0 = (CollisionObject) manifold.getBody0();
                CollisionObject colObj1 = (CollisionObject) manifold.getBody1();

                if (colObj0 == null || colObj1 == null) continue;

                // todo: check sleeping conditions!
                if (colObj0.getActivationState() != CollisionObject.ISLAND_SLEEPING ||
                        colObj1.getActivationState() != CollisionObject.ISLAND_SLEEPING) {

                    // kinematic objects don't merge islands, but wake up all connected objects
                    if (colObj0.isKinematicObject() && colObj0.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                        colObj1.activate();
                    }
                    if (colObj1.isKinematicObject() && colObj1.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                        colObj0.activate();
                    }
                    //filtering for response
                    if (dispatcher.needsResponse(colObj0, colObj1)) {
                        islandManifold.add(manifold);
                    }
                }
            }
        } finally {
            BulletStats.popProfile();
        }
    }

    public void buildAndProcessIslands(Dispatcher dispatcher, ObjectArrayList<CollisionObject> collisionObjects, IslandCallback callback) {
        buildIslands(dispatcher, collisionObjects);

        int endIslandIndex = 1;
        int startIslandIndex;
        int numElem = getUnionFind().getNumElements();

        BulletStats.pushProfile("processIslands");
        try {

            int numManifolds = islandManifold.size();

            // we should do radix sort, it it much faster (O(n) instead of O (n log2(n))
            //islandmanifold.heapSort(btPersistentManifoldSortPredicate());

            // JAVA NOTE: memory optimized sorting with caching of temporary array
            //Collections.sort(islandmanifold, persistentManifoldComparator);
            MiscUtil.quickSort(islandManifold, persistentManifoldComparator);

            // now process all active islands (sets of manifolds for now)

            int startManifoldIndex = 0;
            int endManifoldIndex = 1;

            //int islandId;

            //printf("Start Islands\n");

            // traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
            int[] stackPos = null;
            for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
                stackPos = Stack.getPosition(stackPos);
                int islandId = getUnionFind().getParent(startIslandIndex);
                boolean islandSleeping = false;

                for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (getUnionFind().getParent(endIslandIndex) == islandId); endIslandIndex++) {
                    int sz = getUnionFind().getRank(endIslandIndex);
                    CollisionObject colObj0 = collisionObjects.getQuick(sz);
                    islandBodies.add(colObj0);
                    if (!colObj0.isActive()) {
                        islandSleeping = true;
                    }
                }

                // find the accompanying contact manifold for this islandId
                int numIslandManifolds = 0;
                int startManifold_idx = -1;

                if (startManifoldIndex < numManifolds) {
                    int curIslandId = getIslandId(islandManifold.getQuick(startManifoldIndex));
                    if (curIslandId == islandId) {
                        startManifold_idx = startManifoldIndex;

                        endManifoldIndex = startManifoldIndex + 1;
                        while ((endManifoldIndex < numManifolds) && (islandId == getIslandId(islandManifold.getQuick(endManifoldIndex)))) {
                            endManifoldIndex++;
                        }
                        // Process the actual simulation, only if not sleeping/deactivated
                        numIslandManifolds = endManifoldIndex - startManifoldIndex;
                    }
                }

                if (!islandSleeping) {
                    callback.processIsland(islandBodies, islandBodies.size(), islandManifold, startManifold_idx, numIslandManifolds, islandId);
                }

                if (numIslandManifolds != 0) {
                    startManifoldIndex = endManifoldIndex;
                }

                islandBodies.clear();
                Stack.reset(stackPos);
            }
        } finally {
            BulletStats.popProfile();
        }
    }

    /// /////////////////////////////////////////////////////////////////////////

    public static abstract class IslandCallback {
        public abstract void processIsland(ObjectArrayList<CollisionObject> bodies, int numBodies, ObjectArrayList<PersistentManifold> manifolds, int manifolds_offset, int numManifolds, int islandId);
    }

    private static final Comparator<PersistentManifold> persistentManifoldComparator = (lhs, rhs) ->
            lhs == rhs ? 0 : getIslandId(lhs) < getIslandId(rhs) ? -1 : +1;

}
