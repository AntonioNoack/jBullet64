package com.bulletphysics.collision.dispatch;

import com.bulletphysics.linearmath.MiscUtil;

import java.util.ArrayList;
import java.util.Comparator;

/**
 * UnionFind calculates connected subsets. Implements weighted Quick Union with
 * path compression.
 *
 * @author jezek2
 */
public class UnionFind {

    // Optimization: could use short ints instead of ints (halving memory, would limit the number of rigid bodies to 64k, sounds reasonable).

    private final ArrayList<Element> elements = new ArrayList<>();

    /**
     * This is a special operation, destroying the content of UnionFind.
     * It sorts the elements, based on island id, in order to make it easy to iterate over islands.
     */
    public void sortIslands() {
        // first store the original body index, and islandId
        int numElements = elements.size();
        for (int i = 0; i < numElements; i++) {
            elements.get(i).id = find(i);
            elements.get(i).sz = i;
        }
        try {
            MiscUtil.sort(elements, elementComparator);
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
        }
    }

    public void reset(int N) {
        allocate(N);

        for (int i = 0; i < N; i++) {
            elements.get(i).id = i;
            elements.get(i).sz = 1;
        }
    }

    public int getNumElements() {
        return elements.size();
    }

    public boolean isRoot(int x) {
        return (x == elements.get(x).id);
    }

    public Element getElement(int index) {
        return elements.get(index);
    }

    public void allocate(int N) {
        MiscUtil.resize(elements, N, Element.class);
    }

    public int find(int p, int q) {
        return (find(p) == find(q)) ? 1 : 0;
    }

    public void unite(int p, int q) {
        int i = find(p), j = find(q);
        if (i == j) return;
        elements.get(i).id = j;
        elements.get(j).sz += elements.get(i).sz;
    }

    public int find(int x) {
        while (x != elements.get(x).id) {
            // not really a reason not to use path compression, and it flattens the trees/improves find performance dramatically
            elements.get(x).id = elements.get(elements.get(x).id).id;
            x = elements.get(x).id;
        }
        return x;
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class Element {
        public int id;
        public int sz;
    }

    private static final Comparator<Element> elementComparator = (o1, o2) -> o1 == o2 ? 0 : Integer.compare(o1.id, o2.id);

}
