package com.bulletphysics.collision.dispatch;

import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.util.ObjectArrayList;

import java.util.Comparator;

/**
 * UnionFind calculates connected subsets. Implements weighted Quick Union with
 * path compression.
 *
 * @author jezek2
 */
public class UnionFind {

    // Optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies to 64k, sounds reasonable).

    private final ObjectArrayList<Element> elements = new ObjectArrayList<Element>();

    /**
     * This is a special operation, destroying the content of UnionFind.
     * It sorts the elements, based on island id, in order to make it easy to iterate over islands.
     */
    public void sortIslands() {
        // first store the original body index, and islandId
        int numElements = elements.size();

        for (int i = 0; i < numElements; i++) {
            elements.getQuick(i).id = find(i);
            elements.getQuick(i).sz = i;
        }

        // Sort the vector using predicate and std::sort
        //std::sort(m_elements.begin(), m_elements.end(), btUnionFindElementSortPredicate);
        //perhaps use radix sort?
        //elements.heapSort(btUnionFindElementSortPredicate());

        //Collections.sort(elements);
        MiscUtil.quickSort(elements, elementComparator);
    }

    public void reset(int N) {
        allocate(N);

        for (int i = 0; i < N; i++) {
            elements.getQuick(i).id = i;
            elements.getQuick(i).sz = 1;
        }
    }

    public int getNumElements() {
        return elements.size();
    }

    public boolean isRoot(int x) {
        return (x == elements.getQuick(x).id);
    }

    public Element getElement(int index) {
        return elements.getQuick(index);
    }

    public void allocate(int N) {
        MiscUtil.resize(elements, N, Element.class);
    }

    public void free() {
        elements.clear();
    }

    public int find(int p, int q) {
        return (find(p) == find(q)) ? 1 : 0;
    }

    public void unite(int p, int q) {
        int i = find(p), j = find(q);
        if (i == j) {
            return;
        }

        //#ifndef USE_PATH_COMPRESSION
        ////weighted quick union, this keeps the 'trees' balanced, and keeps performance of unite O( log(n) )
        //if (m_elements[i].m_sz < m_elements[j].m_sz)
        //{
        //	m_elements[i].m_id = j; m_elements[j].m_sz += m_elements[i].m_sz;
        //}
        //else
        //{
        //	m_elements[j].m_id = i; m_elements[i].m_sz += m_elements[j].m_sz;
        //}
        //#else
        elements.getQuick(i).id = j;
        elements.getQuick(j).sz += elements.getQuick(i).sz;
        //#endif //USE_PATH_COMPRESSION
    }

    public int find(int x) {
        //assert(x < m_N);
        //assert(x >= 0);

        while (x != elements.getQuick(x).id) {
            // not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically

            //#ifdef USE_PATH_COMPRESSION
            elements.getQuick(x).id = elements.getQuick(elements.getQuick(x).id).id;
            //#endif //
            x = elements.getQuick(x).id;
            //assert(x < m_N);
            //assert(x >= 0);
        }
        return x;
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class Element {
        public int id;
        public int sz;
    }

    private static final Comparator<Element> elementComparator = (o1, o2) -> o1.id < o2.id ? -1 : +1;

}
