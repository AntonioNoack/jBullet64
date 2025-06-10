package com.bulletphysics.collision.broadphase;

import com.bulletphysics.BulletStats;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * AxisSweep3Internal is an internal base class that implements sweep and prune.
 * Use concrete implementation {@link AxisSweep3} or {@link AxisSweep3_32}.
 *
 * @author jezek2
 */
public abstract class AxisSweep3Internal extends BroadphaseInterface {

    protected int bpHandleMask;
    protected int handleSentinel;

    protected final Vector3d worldAabbMin = new Vector3d(); // overall system bounds
    protected final Vector3d worldAabbMax = new Vector3d(); // overall system bounds

    protected final Vector3d quantize = new Vector3d();     // scaling factor for quantization

    protected int numHandles;                               // number of active handles
    protected int maxHandles;                               // max number of handles
    protected Handle[] handles;                            // handles pool
    protected int firstFreeHandle;                            // free handles list

    protected EdgeArray[] edges = new EdgeArray[3];      // edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)

    protected OverlappingPairCache pairCache;

    // OverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to OverlappingPairCache.
    protected OverlappingPairCallback userPairCallback = null;

    protected boolean ownsPairCache = false;

    // JAVA NOTE: added
    protected int mask;

    AxisSweep3Internal(Vector3d worldAabbMin, Vector3d worldAabbMax, int handleMask, int handleSentinel, int userMaxHandles/* = 16384*/, OverlappingPairCache pairCache/*=0*/) {
        this.bpHandleMask = handleMask;
        this.handleSentinel = handleSentinel;
        this.pairCache = pairCache;

        int maxHandles = userMaxHandles + 1; // need to add one sentinel handle

        if (this.pairCache == null) {
            this.pairCache = new HashedOverlappingPairCache();
            ownsPairCache = true;
        }

        //assert(bounds.HasVolume());

        // init bounds
        this.worldAabbMin.set(worldAabbMin);
        this.worldAabbMax.set(worldAabbMax);

        Vector3d aabbSize = Stack.newVec();
        aabbSize.sub(this.worldAabbMax, this.worldAabbMin);

        int maxInt = this.handleSentinel;

        quantize.set(maxInt / aabbSize.x, maxInt / aabbSize.y, maxInt / aabbSize.z);

        // allocate handles buffer and put all handles on free list
        handles = new Handle[maxHandles];
        for (int i = 0; i < maxHandles; i++) {
            handles[i] = createHandle();
        }
        this.maxHandles = maxHandles;
        this.numHandles = 0;

        // handle 0 is reserved as the null index, and is also used as the sentinel
        firstFreeHandle = 1;
        {
            for (int i = firstFreeHandle; i < maxHandles; i++) {
                handles[i].setNextFree(i + 1);
            }
            handles[maxHandles - 1].setNextFree(0);
        }

        {
            // allocate edge buffers
            for (int i = 0; i < 3; i++) {
                edges[i] = createEdgeArray(maxHandles * 2);
            }
        }
        //removed overlap management

        // make boundary sentinels

        handles[0].clientObject = null;

        for (int axis = 0; axis < 3; axis++) {
            handles[0].setMinEdges(axis, 0);
            handles[0].setMaxEdges(axis, 1);

            edges[axis].setPos(0, 0);
            edges[axis].setHandle(0, 0);
            edges[axis].setPos(1, handleSentinel);
            edges[axis].setHandle(1, 0);
            //#ifdef DEBUG_BROADPHASE
            //debugPrintAxis(axis);
            //#endif //DEBUG_BROADPHASE
        }

        // JAVA NOTE: added
        mask = getMask();
    }

    // allocation/deallocation
    protected int allocHandle() {
        assert (firstFreeHandle != 0);

        int handle = firstFreeHandle;
        firstFreeHandle = getHandle(handle).getNextFree();
        numHandles++;

        return handle;
    }

    protected void freeHandle(int handle) {
        assert (handle > 0 && handle < maxHandles);

        getHandle(handle).setNextFree(firstFreeHandle);
        firstFreeHandle = handle;

        numHandles--;
    }

    protected boolean testOverlap(int ignoreAxis, Handle pHandleA, Handle pHandleB) {
        // optimization 1: check the array index (memory address), instead of the m_pos

        for (int axis = 0; axis < 3; axis++) {
            if (axis == ignoreAxis) continue;
            if (pHandleA.getMaxEdges(axis) < pHandleB.getMinEdges(axis) ||
                    pHandleB.getMaxEdges(axis) < pHandleA.getMinEdges(axis)) {
                return false;
            }
        }

        //optimization 2: only 2 axis need to be tested (conflicts with 'delayed removal' optimization)

		/*for (int axis = 0; axis < 3; axis++)
		{
		if (m_pEdges[axis][pHandleA->m_maxEdges[axis]].m_pos < m_pEdges[axis][pHandleB->m_minEdges[axis]].m_pos ||
		m_pEdges[axis][pHandleB->m_maxEdges[axis]].m_pos < m_pEdges[axis][pHandleA->m_minEdges[axis]].m_pos)
		{
		return false;
		}
		}
		*/

        return true;
    }

    //#ifdef DEBUG_BROADPHASE
    //void debugPrintAxis(int axis,bool checkCardinality=true);
    //#endif //DEBUG_BROADPHASE

    protected void quantize(int[] out, Vector3d point, int isMax) {
        Vector3d clampedPoint = Stack.newVec(point);

        VectorUtil.setMax(clampedPoint, worldAabbMin);
        VectorUtil.setMin(clampedPoint, worldAabbMax);

        clampedPoint.sub(clampedPoint, worldAabbMin);
        VectorUtil.mul(clampedPoint, clampedPoint, quantize);

        out[0] = (((int) clampedPoint.x & bpHandleMask) | isMax) & mask;
        out[1] = (((int) clampedPoint.y & bpHandleMask) | isMax) & mask;
        out[2] = (((int) clampedPoint.z & bpHandleMask) | isMax) & mask;
        Stack.subVec(1);
    }

    // sorting a min edge downwards can only ever *add* overlaps
    protected void sortMinDown(int axis, int edge, boolean updateOverlaps) {
        EdgeArray edgeArray = edges[axis];
        int edgeIdx = edge;
        int prevIdx = edgeIdx - 1;

        Handle edgeHandle = getHandle(edgeArray.getHandle(edgeIdx));

        while (edgeArray.getPos(edgeIdx) < edgeArray.getPos(prevIdx)) {
            Handle prevHandle = getHandle(edgeArray.getHandle(prevIdx));

            if (edgeArray.isMax(prevIdx) != 0) {
                // if previous edge is a maximum check the bounds and add an overlap if necessary
                if (updateOverlaps && testOverlap(axis, edgeHandle, prevHandle)) {
                    pairCache.addOverlappingPair(edgeHandle, prevHandle);
                    if (userPairCallback != null) {
                        userPairCallback.addOverlappingPair(edgeHandle, prevHandle);
                        //AddOverlap(pEdge->m_handle, pPrev->m_handle);
                    }
                }

                // update edge reference in other handle
                prevHandle.incMaxEdges(axis);
            } else {
                prevHandle.incMinEdges(axis);
            }
            edgeHandle.decMinEdges(axis);

            // swap the edges
            edgeArray.swap(edgeIdx, prevIdx);

            // decrement
            edgeIdx--;
            prevIdx--;
        }

        //#ifdef DEBUG_BROADPHASE
        //debugPrintAxis(axis);
        //#endif //DEBUG_BROADPHASE
    }

    // sorting a min edge upwards can only ever *remove* overlaps
    protected void sortMinUp(int axis, int edge, Dispatcher dispatcher, boolean updateOverlaps) {
        EdgeArray edgeArray = edges[axis];
        int edgeIdx = edge;
        int nextIdx = edgeIdx + 1;
        Handle edgeHandle = getHandle(edgeArray.getHandle(edgeIdx));

        while (edgeArray.getHandle(nextIdx) != 0 && (edgeArray.getPos(edgeIdx) >= edgeArray.getPos(nextIdx))) {
            Handle nextHandle = getHandle(edgeArray.getHandle(nextIdx));

            if (edgeArray.isMax(nextIdx) != 0) {
                // if next edge is maximum remove any overlap between the two handles
                if (updateOverlaps) {
                    Handle handle0 = getHandle(edgeArray.getHandle(edgeIdx));
                    Handle handle1 = getHandle(edgeArray.getHandle(nextIdx));

                    pairCache.removeOverlappingPair(handle0, handle1, dispatcher);
                    if (userPairCallback != null) {
                        userPairCallback.removeOverlappingPair(handle0, handle1, dispatcher);
                    }
                }

                // update edge reference in other handle
                nextHandle.decMaxEdges(axis);
            } else {
                nextHandle.decMinEdges(axis);
            }
            edgeHandle.incMinEdges(axis);

            // swap the edges
            edgeArray.swap(edgeIdx, nextIdx);

            // increment
            edgeIdx++;
            nextIdx++;
        }
    }

    // sorting a max edge downwards can only ever *remove* overlaps
    protected void sortMaxDown(int axis, int edge, Dispatcher dispatcher, boolean updateOverlaps) {
        EdgeArray edgeArray = edges[axis];
        int edgeIdx = edge;
        int prevIdx = edgeIdx - 1;
        Handle edgeHandle = getHandle(edgeArray.getHandle(edgeIdx));

        while (edgeArray.getPos(edgeIdx) < edgeArray.getPos(prevIdx)) {
            Handle prevHandle = getHandle(edgeArray.getHandle(prevIdx));

            if (edgeArray.isMax(prevIdx) == 0) {
                // if previous edge was a minimum remove any overlap between the two handles
                if (updateOverlaps) {
                    // this is done during the overlappingpairarray iteration/narrowphase collision
                    Handle handle0 = getHandle(edgeArray.getHandle(edgeIdx));
                    Handle handle1 = getHandle(edgeArray.getHandle(prevIdx));
                    pairCache.removeOverlappingPair(handle0, handle1, dispatcher);
                    if (userPairCallback != null) {
                        userPairCallback.removeOverlappingPair(handle0, handle1, dispatcher);
                    }
                }

                // update edge reference in other handle
                prevHandle.incMinEdges(axis);
            } else {
                prevHandle.incMaxEdges(axis);
            }
            edgeHandle.decMaxEdges(axis);

            // swap the edges
            edgeArray.swap(edgeIdx, prevIdx);

            // decrement
            edgeIdx--;
            prevIdx--;
        }

        //#ifdef DEBUG_BROADPHASE
        //debugPrintAxis(axis);
        //#endif //DEBUG_BROADPHASE
    }

    /**
     * sorting a max edge upwards can only ever *add* overlaps
     */
    private void sortMaxUp(int axis, int edge, boolean updateOverlaps) {
        EdgeArray edgeArray = edges[axis];
        int edgeIdx = edge;
        int prevIdx = edgeIdx + 1;
        Handle edgeHandle = getHandle(edgeArray.getHandle(edgeIdx));

        while (edgeArray.getHandle(prevIdx) != 0 && (edgeArray.getPos(edgeIdx) >= edgeArray.getPos(prevIdx))) {
            Handle nextHandle = getHandle(edgeArray.getHandle(prevIdx));

            if (edgeArray.isMax(prevIdx) == 0) {
                // if next edge is a minimum check the bounds and add an overlap if necessary
                if (updateOverlaps && testOverlap(axis, edgeHandle, nextHandle)) {
                    Handle handle0 = getHandle(edgeArray.getHandle(edgeIdx));
                    Handle handle1 = getHandle(edgeArray.getHandle(prevIdx));
                    pairCache.addOverlappingPair(handle0, handle1);
                    if (userPairCallback != null) {
                        userPairCallback.addOverlappingPair(handle0, handle1);
                    }
                }

                // update edge reference in other handle
                nextHandle.decMinEdges(axis);
            } else {
                nextHandle.decMaxEdges(axis);
            }
            edgeHandle.incMaxEdges(axis);

            // swap the edges
            edgeArray.swap(edgeIdx, prevIdx);

            // increment
            edgeIdx++;
            prevIdx++;
        }
    }

    @SuppressWarnings("unused")
    public int getNumHandles() {
        return numHandles;
    }

    public void calculateOverlappingPairs(Dispatcher dispatcher) {
        if (pairCache.hasDeferredRemoval()) {
            ObjectArrayList<BroadphasePair> overlappingPairArray = pairCache.getOverlappingPairArray();

            // perform a sort, to find duplicates and to sort 'invalid' pairs to the end
            MiscUtil.quickSort(overlappingPairArray, BroadphasePair.broadphasePairSortPredicate);

            BroadphasePair previousPair = new BroadphasePair();
            previousPair.proxy0 = null;
            previousPair.proxy1 = null;
            previousPair.algorithm = null;

            for (int i = 0; i < overlappingPairArray.getSize(); i++) {
                BroadphasePair pair = overlappingPairArray.getQuick(i);

                boolean isDuplicate = pair.equals(previousPair);
                previousPair.set(pair);

                if (isDuplicate || !testAabbOverlap(pair.proxy0, pair.proxy1)) {
                    pairCache.cleanOverlappingPair(pair, dispatcher);
                    pair.proxy0 = null;
                    pair.proxy1 = null;
                    BulletStats.overlappingPairs--;
                }
            }

            overlappingPairArray.removeIf(pair -> pair.proxy0 == null);

            //printf("overlappingPairArray.getSize()=%d\n",overlappingPairArray.getSize());
        }
    }

    public int addHandle(Vector3d aabbMin, Vector3d aabbMax, Object pOwner, short collisionFilterGroup, short collisionFilterMask, Dispatcher dispatcher, Object multiSapProxy) {
        // quantize the bounds
        int[] min = new int[3], max = new int[3];
        quantize(min, aabbMin, 0);
        quantize(max, aabbMax, 1);

        // allocate a handle
        int handle = allocHandle();

        Handle pHandle = getHandle(handle);

        pHandle.uniqueId = handle;
        //pHandle->m_pOverlaps = 0;
        pHandle.clientObject = pOwner;
        pHandle.collisionFilterGroup = collisionFilterGroup;
        pHandle.collisionFilterMask = collisionFilterMask;
        pHandle.multiSapParentProxy = multiSapProxy;

        // compute current limit of edge arrays
        int limit = numHandles * 2;

        // insert new edges just inside the max boundary edge
        for (int axis = 0; axis < 3; axis++) {
            handles[0].setMaxEdges(axis, handles[0].getMaxEdges(axis) + 2);

            edges[axis].set(limit + 1, limit - 1);

            edges[axis].setPos(limit - 1, min[axis]);
            edges[axis].setHandle(limit - 1, handle);

            edges[axis].setPos(limit, max[axis]);
            edges[axis].setHandle(limit, handle);

            pHandle.setMinEdges(axis, limit - 1);
            pHandle.setMaxEdges(axis, limit);
        }

        // now sort the new edges to their correct position
        sortMinDown(0, pHandle.getMinEdges(0), false);
        sortMaxDown(0, pHandle.getMaxEdges(0), dispatcher, false);
        sortMinDown(1, pHandle.getMinEdges(1), false);
        sortMaxDown(1, pHandle.getMaxEdges(1), dispatcher, false);
        sortMinDown(2, pHandle.getMinEdges(2), true);
        sortMaxDown(2, pHandle.getMaxEdges(2), dispatcher, true);

        return handle;
    }

    public void removeHandle(int handleIdx, Dispatcher dispatcher) {
        Handle handle = getHandle(handleIdx);

        // explicitly remove the pairs containing the proxy
        // we could do it also in the sortMinUp (passing true)
        // todo: compare performance
        if (!pairCache.hasDeferredRemoval()) {
            pairCache.removeOverlappingPairsContainingProxy(handle, dispatcher);
        }

        // compute current limit of edge arrays
        int limit = numHandles * 2;

        int axis;

        for (axis = 0; axis < 3; axis++) {
            handles[0].setMaxEdges(axis, handles[0].getMaxEdges(axis) - 2);
        }

        // remove the edges by sorting them up to the end of the list
        for (axis = 0; axis < 3; axis++) {
            EdgeArray pEdges = this.edges[axis];
            int max = handle.getMaxEdges(axis);
            pEdges.setPos(max, handleSentinel);

            sortMaxUp(axis, max, false);

            int i = handle.getMinEdges(axis);
            pEdges.setPos(i, handleSentinel);

            sortMinUp(axis, i, dispatcher, false);

            pEdges.setHandle(limit - 1, 0);
            pEdges.setPos(limit - 1, handleSentinel);

            //#ifdef DEBUG_BROADPHASE
            //debugPrintAxis(axis,false);
            //#endif //DEBUG_BROADPHASE
        }

        // free the handle
        freeHandle(handleIdx);
    }

    public void updateHandle(int handleIndex, Vector3d aabbMin, Vector3d aabbMax, Dispatcher dispatcher) {
        Handle handle = getHandle(handleIndex);

        // quantize the new bounds
        int[] min = new int[3], max = new int[3];
        quantize(min, aabbMin, 0);
        quantize(max, aabbMax, 1);

        // update changed edges
        for (int axis = 0; axis < 3; axis++) {
            int emin = handle.getMinEdges(axis);
            int emax = handle.getMaxEdges(axis);

            int dmin = min[axis] - edges[axis].getPos(emin);
            int dmax = max[axis] - edges[axis].getPos(emax);

            edges[axis].setPos(emin, min[axis]);
            edges[axis].setPos(emax, max[axis]);

            // expand (only adds overlaps)
            if (dmin < 0) {
                sortMinDown(axis, emin, true);
            }
            if (dmax > 0) {
                sortMaxUp(axis, emax, true); // shrink (only removes overlaps)
            }
            if (dmin > 0) {
                sortMinUp(axis, emin, dispatcher, true);
            }
            if (dmax < 0) {
                sortMaxDown(axis, emax, dispatcher, true);
            }

            //#ifdef DEBUG_BROADPHASE
            //debugPrintAxis(axis);
            //#endif //DEBUG_BROADPHASE
        }
    }

    private Handle getHandle(int index) {
        return handles[index];
    }

    public BroadphaseProxy createProxy(
            Vector3d aabbMin, Vector3d aabbMax, BroadphaseNativeType shapeType, Object userPtr,
            short collisionFilterGroup, short collisionFilterMask, Dispatcher dispatcher, Object multiSapProxy) {
        int handleId = addHandle(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy);
        return getHandle(handleId);
    }

    public void destroyProxy(BroadphaseProxy proxy, Dispatcher dispatcher) {
        Handle handle = (Handle) proxy;
        removeHandle(handle.uniqueId, dispatcher);
    }

    public void setAabb(BroadphaseProxy proxy, Vector3d aabbMin, Vector3d aabbMax, Dispatcher dispatcher) {
        Handle handle = (Handle) proxy;
        updateHandle(handle.uniqueId, aabbMin, aabbMax, dispatcher);
    }

    public boolean testAabbOverlap(BroadphaseProxy proxy0, BroadphaseProxy proxy1) {
        Handle pHandleA = (Handle) proxy0;
        Handle pHandleB = (Handle) proxy1;

        // optimization 1: check the array index (memory address), instead of the m_pos

        for (int axis = 0; axis < 3; axis++) {
            if (pHandleA.getMaxEdges(axis) < pHandleB.getMinEdges(axis) ||
                    pHandleB.getMaxEdges(axis) < pHandleA.getMinEdges(axis)) {
                return false;
            }
        }
        return true;
    }

    public OverlappingPairCache getOverlappingPairCache() {
        return pairCache;
    }

    @SuppressWarnings("unused")
    public void setOverlappingPairUserCallback(OverlappingPairCallback pairCallback) {
        userPairCallback = pairCallback;
    }

    @SuppressWarnings("unused")
    public OverlappingPairCallback getOverlappingPairUserCallback() {
        return userPairCallback;
    }

    // getAabb returns the axis aligned bounding box in the 'global' coordinate frame
    // will add some transform later
    public void getBroadphaseAabb(Vector3d aabbMin, Vector3d aabbMax) {
        aabbMin.set(worldAabbMin);
        aabbMax.set(worldAabbMax);
    }

    protected abstract EdgeArray createEdgeArray(int size);

    protected abstract Handle createHandle();

    protected abstract int getMask();

    protected interface EdgeArray {
        void swap(int idx1, int idx2);

        void set(int dest, int src);

        int getPos(int index);

        void setPos(int index, int value);

        int getHandle(int index);

        void setHandle(int index, int value);

        default int isMax(int offset) {
            return (getPos(offset) & 1);
        }
    }

    protected static abstract class Handle extends BroadphaseProxy {
        public abstract int getMinEdges(int edgeIndex);

        public abstract void setMinEdges(int edgeIndex, int value);

        public abstract int getMaxEdges(int edgeIndex);

        public abstract void setMaxEdges(int edgeIndex, int value);

        public void incMinEdges(int edgeIndex) {
            setMinEdges(edgeIndex, getMinEdges(edgeIndex) + 1);
        }

        public void incMaxEdges(int edgeIndex) {
            setMaxEdges(edgeIndex, getMaxEdges(edgeIndex) + 1);
        }

        public void decMinEdges(int edgeIndex) {
            setMinEdges(edgeIndex, getMinEdges(edgeIndex) - 1);
        }

        public void decMaxEdges(int edgeIndex) {
            setMaxEdges(edgeIndex, getMaxEdges(edgeIndex) - 1);
        }

        public void setNextFree(int next) {
            setMinEdges(0, next);
        }

        public int getNextFree() {
            return getMinEdges(0);
        }
    }

}
