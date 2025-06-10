package com.bulletphysics.collision.dispatch

import com.bulletphysics.BulletStats.popProfile
import com.bulletphysics.BulletStats.pushProfile
import com.bulletphysics.collision.broadphase.Dispatcher
import com.bulletphysics.collision.narrowphase.PersistentManifold
import com.bulletphysics.linearmath.MiscUtil
import com.bulletphysics.util.ObjectArrayList
import cz.advel.stack.Stack

/**
 * SimulationIslandManager creates and handles simulation islands, using [UnionFind].
 *
 * @author jezek2
 */
class SimulationIslandManager {
    val unionFind: UnionFind = UnionFind()

    private val islandManifold = ObjectArrayList<PersistentManifold>()
    private val islandBodies = ObjectArrayList<CollisionObject>()

    fun initUnionFind(n: Int) {
        unionFind.reset(n)
    }

    fun findUnions(dispatcher: Dispatcher?, colWorld: CollisionWorld) {
        val pairPtr = colWorld.pairCache.overlappingPairArray
        for (i in pairPtr.indices) {
            val collisionPair = pairPtr.getQuick(i)

            val colObj0 = collisionPair.proxy0?.clientObject as CollisionObject?
            val colObj1 = collisionPair.proxy1?.clientObject as CollisionObject?

            if (((colObj0 != null) && ((colObj0).mergesSimulationIslands())) &&
                ((colObj1 != null) && ((colObj1).mergesSimulationIslands()))
            ) {
                unionFind.combineIslands((colObj0).islandTag, (colObj1).islandTag)
            }
        }
    }

    fun updateActivationState(colWorld: CollisionWorld, dispatcher: Dispatcher?) {
        initUnionFind(colWorld.collisionObjectArray.size)

        // put the index into m_controllers into m_tag
        run {
            var index = 0
            var i: Int
            i = 0
            while (i < colWorld.collisionObjectArray.size) {
                val collisionObject = colWorld.collisionObjectArray.getQuick(i)
                collisionObject.islandTag = index
                collisionObject.companionId = -1
                collisionObject.hitFraction = 1.0
                index++
                i++
            }
        }

        // do the union find
        findUnions(dispatcher, colWorld)
    }

    fun storeIslandActivationState(colWorld: CollisionWorld) {
        // put the islandId ('find' value) into m_tag
        for (i in colWorld.collisionObjectArray.indices) {
            val collisionObject = colWorld.collisionObjectArray.getQuick(i)
            if (!collisionObject.isStaticOrKinematicObject) {
                collisionObject.islandTag = unionFind.findGroupId(i)
                collisionObject.companionId = -1
            } else {
                collisionObject.islandTag = -1
                collisionObject.companionId = -2
            }
        }
    }

    fun buildIslands(dispatcher: Dispatcher, collisionObjects: ObjectArrayList<CollisionObject>) {
        pushProfile("islandUnionFindAndQuickSort")
        try {
            islandManifold.clear()

            // we are going to sort the unionfind array, and store the element id in the size
            // afterward, we clean unionfind, to make sure no-one uses it anymore
            this.unionFind.sortIslands()
            val numElem = this.unionFind.numElements

            var endIslandIndex: Int
            var startIslandIndex: Int

            // update the sleeping state for bodies, if all are sleeping
            startIslandIndex = 0
            while (startIslandIndex < numElem) {
                val islandId = this.unionFind.getParent(startIslandIndex)

                endIslandIndex = startIslandIndex + 1
                while ((endIslandIndex < numElem) && (this.unionFind.getParent(endIslandIndex) == islandId)) {
                    endIslandIndex++
                }

                var allSleeping = true

                var idx: Int
                idx = startIslandIndex
                while (idx < endIslandIndex) {
                    val sz = this.unionFind.getRank(idx)

                    val colObj0 = collisionObjects.getQuick(sz)

                    assert((colObj0.islandTag == islandId) || (colObj0.islandTag == -1))
                    if (colObj0.islandTag == islandId) {
                        if (colObj0.activationState == CollisionObject.ACTIVE_TAG) {
                            allSleeping = false
                        }
                        if (colObj0.activationState == CollisionObject.DISABLE_DEACTIVATION) {
                            allSleeping = false
                        }
                    }
                    idx++
                }


                if (allSleeping) {
                    //int idx;
                    idx = startIslandIndex
                    while (idx < endIslandIndex) {
                        val sz = this.unionFind.getRank(idx)
                        val colObj0 = collisionObjects.getQuick(sz)

                        assert((colObj0.islandTag == islandId) || (colObj0.islandTag == -1))

                        if (colObj0.islandTag == islandId) {
                            colObj0.setActivationStateMaybe(CollisionObject.ISLAND_SLEEPING)
                        }
                        idx++
                    }
                } else {
                    //int idx;

                    idx = startIslandIndex
                    while (idx < endIslandIndex) {
                        val i = this.unionFind.getRank(idx)

                        val colObj0 = collisionObjects.getQuick(i)

                        assert((colObj0.islandTag == islandId) || (colObj0.islandTag == -1))
                        if (colObj0.islandTag == islandId) {
                            if (colObj0.activationState == CollisionObject.ISLAND_SLEEPING) {
                                colObj0.setActivationStateMaybe(CollisionObject.WANTS_DEACTIVATION)
                            }
                        }
                        idx++
                    }
                }
                startIslandIndex = endIslandIndex
            }


            var i: Int
            val maxNumManifolds = dispatcher.numManifolds

            //#define SPLIT_ISLANDS 1
            //#ifdef SPLIT_ISLANDS
            //#endif //SPLIT_ISLANDS
            i = 0
            while (i < maxNumManifolds) {
                val manifold = dispatcher.getManifoldByIndexInternal(i)

                val colObj0 = manifold.getBody0() as CollisionObject?
                val colObj1 = manifold.getBody1() as CollisionObject?

                if (colObj0 == null || colObj1 == null) {
                    i++
                    continue
                }

                // todo: check sleeping conditions!
                if (colObj0.activationState != CollisionObject.ISLAND_SLEEPING ||
                    colObj1.activationState != CollisionObject.ISLAND_SLEEPING
                ) {
                    // kinematic objects don't merge islands, but wake up all connected objects

                    if (colObj0.isKinematicObject && colObj0.activationState != CollisionObject.ISLAND_SLEEPING) {
                        colObj1.activate()
                    }
                    if (colObj1.isKinematicObject && colObj1.activationState != CollisionObject.ISLAND_SLEEPING) {
                        colObj0.activate()
                    }
                    //filtering for response
                    if (dispatcher.needsResponse(colObj0, colObj1)) {
                        islandManifold.add(manifold)
                    }
                }
                i++
            }
        } finally {
            popProfile()
        }
    }

    fun buildAndProcessIslands(
        dispatcher: Dispatcher,
        collisionObjects: ObjectArrayList<CollisionObject>,
        callback: IslandCallback
    ) {
        buildIslands(dispatcher, collisionObjects)

        var endIslandIndex: Int
        var startIslandIndex: Int
        val numElem = this.unionFind.numElements

        pushProfile("processIslands")
        try {
            val numManifolds = islandManifold.size

            // we should do radix sort, it it much faster (O(n) instead of O (n log2(n))
            //islandmanifold.heapSort(btPersistentManifoldSortPredicate());

            // JAVA NOTE: memory optimized sorting with caching of temporary array
            //Collections.sort(islandmanifold, persistentManifoldComparator);
            MiscUtil.quickSort(islandManifold, sortByIslandId)

            // now process all active islands (sets of manifolds for now)
            var startManifoldIndex = 0
            var endManifoldIndex = 1

            //int islandId;

            //printf("Start Islands\n");

            // traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
            var stackPos: IntArray? = null
            startIslandIndex = 0
            while (startIslandIndex < numElem) {
                stackPos = Stack.getPosition(stackPos)
                val islandId = this.unionFind.getParent(startIslandIndex)
                var islandSleeping = false

                endIslandIndex = startIslandIndex
                while ((endIslandIndex < numElem) && (this.unionFind.getParent(endIslandIndex) == islandId)) {
                    val sz = this.unionFind.getRank(endIslandIndex)
                    val colObj0 = collisionObjects.getQuick(sz)
                    islandBodies.add(colObj0)
                    if (!colObj0.isActive) {
                        islandSleeping = true
                    }
                    endIslandIndex++
                }

                // find the accompanying contact manifold for this islandId
                var numIslandManifolds = 0
                var startManifoldIdx = -1

                if (startManifoldIndex < numManifolds) {
                    val curIslandId: Int = Companion.getIslandId(islandManifold.getQuick(startManifoldIndex)!!)
                    if (curIslandId == islandId) {
                        startManifoldIdx = startManifoldIndex

                        endManifoldIndex = startManifoldIndex + 1
                        while ((endManifoldIndex < numManifolds) && (islandId == Companion.getIslandId(
                                islandManifold.getQuick(
                                    endManifoldIndex
                                )!!
                            ))
                        ) {
                            endManifoldIndex++
                        }
                        // Process the actual simulation, only if not sleeping/deactivated
                        numIslandManifolds = endManifoldIndex - startManifoldIndex
                    }
                }

                if (!islandSleeping) {
                    callback.processIsland(
                        islandBodies,
                        islandBodies.size,
                        islandManifold,
                        startManifoldIdx,
                        numIslandManifolds,
                        islandId
                    )
                }

                if (numIslandManifolds != 0) {
                    startManifoldIndex = endManifoldIndex
                }

                islandBodies.clear()
                Stack.reset(stackPos)
                startIslandIndex = endIslandIndex
            }
        } finally {
            popProfile()
        }
    }

    /** ///////////////////////////////////////////////////////////////////////// */
    abstract class IslandCallback {
        abstract fun processIsland(
            bodies: ObjectArrayList<CollisionObject>, numBodies: Int,
            manifolds: ObjectArrayList<PersistentManifold>, manifoldsOffset: Int, numManifolds: Int,
            islandId: Int
        )
    }

    companion object {
        private fun getIslandId(lhs: PersistentManifold): Int {
            val islandId: Int
            val obj0 = lhs.getBody0() as CollisionObject
            val obj1 = lhs.getBody1() as CollisionObject
            islandId = if (obj0.islandTag >= 0) obj0.islandTag else obj1.islandTag
            return islandId
        }

        private val sortByIslandId = Comparator.comparingInt<PersistentManifold>(::getIslandId)
    }
}
