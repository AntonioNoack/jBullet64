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
                list.swapRemove(list.size() - 1);
            }
        } catch (IllegalAccessException | InstantiationException e) {
            throw new IllegalStateException(e);
        }
    }

    public static double GEN_clamped(double a, double lb, double ub) {
        return a < lb ? lb : Math.min(ub, a);
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
