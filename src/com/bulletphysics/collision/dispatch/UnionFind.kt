package com.bulletphysics.collision.dispatch;

import com.bulletphysics.extras.gimpact.IntPairList;
import com.bulletphysics.linearmath.LongComparator;
import com.bulletphysics.linearmath.MiscUtil;

/**
 * UnionFind algorithm calculates connected subsets. Implements weighted Quick Union with path compression.
 * <a href="https://www.geeksforgeeks.org/union-by-rank-and-path-compression-in-union-find-algorithm/">Union-Find Algorithm</a>
 *
 * @author jezek2
 */
public class UnionFind {

    // first = parent, second = rank
    private final IntPairList elements = new IntPairList();

    // helper methods to avoid hundreds of helper objects
    private void set(int i, int parent, int rank) {
        elements.setPair(i, parent, rank);
    }

    public int getParent(int i) {
        return elements.getFirst(i);
    }

    public int getRank(int i) {
        return elements.getSecond(i);
    }

    private void setParent(int i, int parent) {
        elements.setPair(i, parent, getRank(i));
    }

    private void setRank(int i, int rank) {
        elements.setPair(i, getParent(i), rank);
    }


    /**
     * This is a special operation, destroying the content of UnionFind.
     * It sorts the elements, based on island id, in order to make it easy to iterate over islands.
     */
    public void sortIslands() {
        // first store the original body index, and islandId
        int numElements = elements.size();
        for (int i = 0; i < numElements; i++) {
            elements.setPair(i, findGroupId(i), i);
        }
        MiscUtil.quickSort(elements, sortByParent);
    }

    public void reset(int N) {
        ensureSize(N);

        for (int i = 0; i < N; i++) {
            set(i, i, 1);
        }
    }

    public int getNumElements() {
        return elements.size();
    }

    private void ensureSize(int N) {
        elements.resize(N);
        elements.size = N;
    }

    /**
     * Combine p and q.
     * Weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O(log(n))
     */
    public void combineIslands(int p, int q) {
        int i = findGroupId(p), j = findGroupId(q);
        if (i == j) {
            return;
        }

        int ir = getRank(i);
        int jr = getRank(j);
        if (ir < jr) {
            setParent(i, j);
        } else if (ir > jr) {
            setParent(j, i);
        } else {
            setParent(j, i);
            setRank(i, ir + jr);
        }
    }

    /**
     * Finds group ID. Will be unique for all connected nodes,
     * and different for not-connected nodes.
     * <br>
     * Will be ID of one of the member nodes. Which one is kind of random.
     */
    public int findGroupId(int nodeId) {
        while (true) {
            int parentId = getParent(nodeId);
            if (nodeId != parentId) {
                // links to other node -> not self ->
                // - check parent
                // - mark grandparent as new parent for quicker future access
                int grandParentId = getParent(parentId);
                setParent(nodeId, grandParentId);
                nodeId = grandParentId;
            } else {
                return nodeId;
            }
        }
    }

    /**
     * Compare/Sort by parent (parent is the groupId after flattening/"sorting").
     * Parent is in the high bits, and the highest bit shouldn't be taken.
     * So we can just compare the values as-is.
     */
    private static final LongComparator sortByParent = Long::compare;

}
