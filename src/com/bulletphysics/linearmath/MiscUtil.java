package com.bulletphysics.linearmath;

import com.bulletphysics.util.DoubleArrayList;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectArrayList;

import java.util.Comparator;

/**
 * Miscellaneous utility functions.
 *
 * @author jezek2
 */
public class MiscUtil {

    public static int getListCapacityForHash(ObjectArrayList<?> list) {
        return getListCapacityForHash(list.size());
    }

    public static int getListCapacityForHash(int size) {
        int n = 2;
        while (n < size) {
            n <<= 1;
        }
        return n;
    }

    /**
     * Ensures valid index in provided list by filling list with provided values
     * until the index is valid.
     */
    public static <T> void ensureIndex(ObjectArrayList<T> list, int index, T value) {
        while (list.size() <= index) {
            list.add(value);
        }
    }

    /**
     * Resizes list to exact size, filling with given value when expanding.
     */
    public static void resize(IntArrayList list, int size, int value) {
        while (list.size() < size) {
            list.add(value);
        }

        while (list.size() > size) {
            list.remove(list.size() - 1);
        }
    }

    /**
     * Resizes list to exact size, filling with given value when expanding.
     */
    public static void resize(DoubleArrayList list, int size, double value) {
        while (list.size() < size) {
            list.add(value);
        }

        while (list.size() > size) {
            list.remove(list.size() - 1);
        }
    }

    /**
     * Resizes list to exact size, filling with new instances of given class type
     * when expanding.
     */
    public static <T> void resize(ObjectArrayList<T> list, int size, Class<T> valueCls) {
        try {
            while (list.size() < size) {
                list.add(valueCls != null ? valueCls.newInstance() : null);
            }

            while (list.size() > size) {
                list.removeQuick(list.size() - 1);
            }
        } catch (IllegalAccessException | InstantiationException e) {
            throw new IllegalStateException(e);
        }
    }

    /**
     * Searches object in array.
     *
     * @return first index of match, or -1 when not found
     */
    public static <T> int indexOf(T[] array, T obj) {
        for (int i = 0; i < array.length; i++) {
            if (array[i] == obj) return i;
        }
        return -1;
    }

    public static double GEN_clamped(double a, double lb, double ub) {
        return a < lb ? lb : Math.min(ub, a);
    }

    private static <T> void downHeap(ObjectArrayList<T> pArr, int k, int n, Comparator<T> comparator) {
        /*  PRE: a[k+1..N] is a heap */
        /* POST:  a[k..N]  is a heap */

        T temp = pArr.getQuick(k - 1);
        /* k has child(s) */
        while (k <= n / 2) {
            int child = 2 * k;

            if ((child < n) && comparator.compare(pArr.getQuick(child - 1), pArr.getQuick(child)) < 0) {
                child++;
            }
            /* pick larger child */
            if (comparator.compare(temp, pArr.getQuick(child - 1)) < 0) {
                /* move child up */
                pArr.setQuick(k - 1, pArr.getQuick(child - 1));
                k = child;
            } else {
                break;
            }
        }
        pArr.setQuick(k - 1, temp);
    }

    /**
     * Sorts list using heap sort.<p>
     * <p>
     * Implementation from: <a href="http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/">...</a>
     */
    public static <T> void heapSort(ObjectArrayList<T> list, Comparator<T> comparator) {
        /* sort a[0..N-1],  N.B. 0 to N-1 */
        int k;
        int n = list.size();
        for (k = n / 2; k > 0; k--) {
            downHeap(list, k, n, comparator);
        }

        /* a[1..N] is now a heap */
        while (n >= 1) {
            swap(list, 0, n - 1); /* largest of a[0..n-1] */

            n = n - 1;
            /* restore a[1..i-1] heap */
            downHeap(list, 1, n, comparator);
        }
    }

    private static <T> void swap(ObjectArrayList<T> list, int index0, int index1) {
        T temp = list.getQuick(index0);
        list.setQuick(index0, list.getQuick(index1));
        list.setQuick(index1, temp);
    }

    /**
     * Sorts list using quick sort.<p>
     */
    public static <T> void quickSort(ObjectArrayList<T> list, Comparator<T> comparator) {
        // don't sort 0 or 1 elements
        if (list.size() > 1) {
            quickSortInternal(list, comparator, 0, list.size() - 1);
        }
    }

    private static <T> void quickSortInternal(ObjectArrayList<T> list, Comparator<T> comparator, int lo, int hi) {
        // lo is the lower index, hi is the upper index
        // of the region of array a that is to be sorted
        int i = lo, j = hi;
        T x = list.getQuick((lo + hi) / 2);

        // partition
        do {
            while (comparator.compare(list.getQuick(i), x) < 0) i++;
            while (comparator.compare(x, list.getQuick(j)) < 0) j--;

            if (i <= j) {
                swap(list, i, j);
                i++;
                j--;
            }
        } while (i <= j);

        // recursion
        if (lo < j) {
            quickSortInternal(list, comparator, lo, j);
        }
        if (i < hi) {
            quickSortInternal(list, comparator, i, hi);
        }
    }

}
