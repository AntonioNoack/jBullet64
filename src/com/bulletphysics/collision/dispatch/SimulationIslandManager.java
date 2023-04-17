package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.linearmath.MiscUtil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * SimulationIslandManager creates and handles simulation islands, using {@link UnionFind}.
 *
 * @author jezek2
 */
public class SimulationIslandManager {

    private final UnionFind unionFind = new UnionFind();

    private final ArrayList<PersistentManifold> islandManifold = new ArrayList<>();
    private final ArrayList<CollisionObject> islandBodies = new ArrayList<>();

    public void initUnionFind(int n) {
        unionFind.reset(n);
    }

    public UnionFind getUnionFind() {
        return unionFind;
    }

    public void findUnions(Dispatcher dispatcher, CollisionWorld colWorld) {
        List<BroadphasePair> pairPtr = colWorld.getPairCache().getOverlappingPairArray();
        for (BroadphasePair collisionPair : pairPtr) {
            CollisionObject colObj0 = (CollisionObject) collisionPair.pProxy0.clientObject;
            CollisionObject colObj1 = (CollisionObject) collisionPair.pProxy1.clientObject;

            if (((colObj0 != null) && ((colObj0).mergesSimulationIslands())) &&
                    ((colObj1 != null) && ((colObj1).mergesSimulationIslands()))) {
                unionFind.unite((colObj0).getIslandTag(), (colObj1).getIslandTag());
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
                CollisionObject collisionObject = colWorld.getCollisionObjectArray().get(i);
                collisionObject.setIslandTag(index);
                collisionObject.setCompanionId(-1);
                collisionObject.setHitFraction(1f);
                index++;
            }
        }
        // do the union find

        findUnions(dispatcher, colWorld);
    }

    public void storeIslandActivationState(CollisionWorld colWorld) {
        // put the islandId ('find' value) into m_tag
        {
            int index = 0;
            int i;
            for (i = 0; i < colWorld.getCollisionObjectArray().size(); i++) {
                CollisionObject collisionObject = colWorld.getCollisionObjectArray().get(i);
                if (!collisionObject.isStaticOrKinematicObject()) {
                    collisionObject.setIslandTag(unionFind.find(index));
                    collisionObject.setCompanionId(-1);
                } else {
                    collisionObject.setIslandTag(-1);
                    collisionObject.setCompanionId(-2);
                }
                index++;
            }
        }
    }

    private static int getIslandId(PersistentManifold lhs) {
        final CollisionObject a = (CollisionObject) lhs.getBody0();
        final CollisionObject b = (CollisionObject) lhs.getBody1();
        final int ai = a.getIslandTag();
        return ai >= 0 ? ai : b.getIslandTag();
    }

    public void buildIslands(Dispatcher dispatcher, ArrayList<CollisionObject> collisionObjects) {

        islandManifold.clear();

        // we are going to sort the unionfind array, and store the element id in the size
        // afterwards, we clean unionfind, to make sure no-one uses it anymore

        getUnionFind().sortIslands();
        int numElem = getUnionFind().getNumElements();

        int endIslandIndex = 1;
        int startIslandIndex;

        // update the sleeping state for bodies, if all are sleeping
        for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
            int islandId = getUnionFind().getElement(startIslandIndex).id;
            endIslandIndex = startIslandIndex + 1;
            while (endIslandIndex < numElem && getUnionFind().getElement(endIslandIndex).id == islandId) {
                endIslandIndex++;
            }

            //int numSleeping = 0;

            boolean allSleeping = true;

            int idx;
            for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                int i = getUnionFind().getElement(idx).sz;

                CollisionObject colObj0 = collisionObjects.get(i);
					/*if ((colObj0.getIslandTag() != islandId) && (colObj0.getIslandTag() != -1)) {
						//System.err.println("error in island management\n");
					}*/

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
                    int i = getUnionFind().getElement(idx).sz;
                    CollisionObject colObj0 = collisionObjects.get(i);
						/*if ((colObj0.getIslandTag() != islandId) && (colObj0.getIslandTag() != -1)) {
							//System.err.println("error in island management\n");
						}*/

                    assert ((colObj0.getIslandTag() == islandId) || (colObj0.getIslandTag() == -1));

                    if (colObj0.getIslandTag() == islandId) {
                        colObj0.setActivationState(CollisionObject.ISLAND_SLEEPING);
                    }
                }
            } else {

                //int idx;
                for (idx = startIslandIndex; idx < endIslandIndex; idx++) {
                    int i = getUnionFind().getElement(idx).sz;

                    CollisionObject colObj0 = collisionObjects.get(i);
						/*if ((colObj0.getIslandTag() != islandId) && (colObj0.getIslandTag() != -1)) {
							//System.err.println("error in island management\n");
						}*/

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

        for (i = 0; i < maxNumManifolds; i++) {
            PersistentManifold manifold = dispatcher.getManifoldByIndexInternal(i);

            CollisionObject colObj0 = (CollisionObject) manifold.getBody0();
            CollisionObject colObj1 = (CollisionObject) manifold.getBody1();

            // todo: check sleeping conditions!
            if (((colObj0 != null) && colObj0.getActivationState() != CollisionObject.ISLAND_SLEEPING) ||
                    ((colObj1 != null) && colObj1.getActivationState() != CollisionObject.ISLAND_SLEEPING)) {

                // kinematic objects don't merge islands, but wake up all connected objects
                if (colObj0 != null && colObj0.isKinematicObject() && colObj0.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                    colObj1.activate();
                }
                if (colObj0 != null && colObj1.isKinematicObject() && colObj1.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                    colObj0.activate();
                }
                if (dispatcher.needsResponse(colObj0, colObj1)) {
                    islandManifold.add(manifold);
                }
            }
        }
    }

    public void buildAndProcessIslands(Dispatcher dispatcher, ArrayList<CollisionObject> collisionObjects, IslandCallback callback) {
        buildIslands(dispatcher, collisionObjects);

        int endIslandIndex;
        int startIslandIndex;
        int numElem = getUnionFind().getNumElements();

        // Sort manifolds, based on islands
        // Sort the vector using predicate and std::sort

        int numManifolds = islandManifold.size();
        synchronized (islandManifold) {
            // todo why is sort saying this is contradicting sort order?
            try {
                MiscUtil.sort(islandManifold, persistentManifoldComparator);
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            }
        }

        // now process all active islands (sets of manifolds for now)

        int startManifoldIndex = 0;
        int endManifoldIndex = 1;

        // traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
        for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex) {
            int islandId = getUnionFind().getElement(startIslandIndex).id;
            boolean islandSleeping = false;

            for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (getUnionFind().getElement(endIslandIndex).id == islandId); endIslandIndex++) {
                int i = getUnionFind().getElement(endIslandIndex).sz;
                CollisionObject colObj0 = collisionObjects.get(i);
                islandBodies.add(colObj0);
                if (!colObj0.isActive()) {
                    islandSleeping = true;
                }
            }

            // find the accompanying contact manifold for this islandId
            int numIslandManifolds = 0;
            int startManifold_idx = -1;

            if (startManifoldIndex < numManifolds) {
                int curIslandId = getIslandId(islandManifold.get(startManifoldIndex));
                if (curIslandId == islandId) {
                    startManifold_idx = startManifoldIndex;
                    endManifoldIndex = startManifoldIndex + 1;
                    while (endManifoldIndex < numManifolds &&
                            islandId == getIslandId(islandManifold.get(endManifoldIndex))) {
                        endManifoldIndex++;
                    }
                    // Process the actual simulation, only if not sleeping/deactivated
                    numIslandManifolds = endManifoldIndex - startManifoldIndex;
                }

            }

            if (!islandSleeping) {
                callback.processIsland(islandBodies, islandBodies.size(), islandManifold, startManifold_idx, numIslandManifolds, islandId);
                //printf("Island callback of size:%d bodies, %d manifolds\n",islandBodies.size(),numIslandManifolds);
            }

            if (numIslandManifolds != 0) {
                startManifoldIndex = endManifoldIndex;
            }

            islandBodies.clear();
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    public static abstract class IslandCallback {
        public abstract void processIsland(ArrayList<CollisionObject> bodies, int numBodies, ArrayList<PersistentManifold> manifolds, int manifolds_offset, int numManifolds, int islandId);
    }

    private static final Comparator<PersistentManifold> persistentManifoldComparator =
            (lhs, rhs) -> lhs == rhs ? 0 : Integer.compare(getIslandId(lhs), getIslandId(rhs));

}
