package com.bulletphysics.util;

import java.lang.reflect.Array;
import java.util.*;

/**
 * Object pool for arrays.
 *
 * @author jezek2
 */
public class ArrayPool<T> {

    private final Class<Object> componentType;
    private final ObjectArrayList<Object> list = new ObjectArrayList<>();
    private final Comparator<Object> comparator;
    private final IntValue key = new IntValue();

    /**
     * Creates object pool.
     *
     * @param componentType
     */
    public ArrayPool(Class componentType) {
        this.componentType = componentType;

        if (componentType == double.class) {
            comparator = floatComparator;
        } else if (componentType == int.class) {
            comparator = intComparator;
        } else if (!componentType.isPrimitive()) {
            comparator = objectComparator;
        } else {
            throw new UnsupportedOperationException("unsupported type " + componentType);
        }
    }

    @SuppressWarnings("unchecked")
    private T create(int length) {
        return (T) Array.newInstance(componentType, length);
    }

    /**
     * Returns array of exactly the same length as demanded, or create one if not
     * present in the pool.
     *
     * @param length
     * @return array
     */
    @SuppressWarnings("unchecked")
    public T getFixed(int length) {
        key.value = length;
        int index = Collections.binarySearch(list, key, comparator);
        if (index < 0) {
            return create(length);
        }
        return (T) list.remove(index);
    }

    /**
     * Returns array that has same or greater length, or create one if not present
     * in the pool.
     *
     * @param length the minimum length required
     * @return array
     */
    @SuppressWarnings("unchecked")
    public T getAtLeast(int length) {
        key.value = length;
        int index = Collections.binarySearch(list, key, comparator);
        if (index < 0) {
            index = -index - 1;
            if (index < list.size()) {
                return (T) list.remove(index);
            } else {
                return create(length);
            }
        }
        return (T) list.remove(index);
    }

    /**
     * Releases array into object pool.
     *
     * @param array previously obtained array from this pool
     */
    @SuppressWarnings("unchecked")
    public void release(T array) {
        int index = Collections.binarySearch(list, array, comparator);
        if (index < 0) index = -index - 1;
        list.add(index, array);

        // remove references from object arrays:
        if (comparator == objectComparator) {
            Object[] objArray = (Object[]) array;
            Arrays.fill(objArray, null);
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    private static final Comparator<Object> floatComparator = (o1, o2) -> {
        int len1 = (o1 instanceof IntValue) ? ((IntValue) o1).value : ((double[]) o1).length;
        int len2 = (o2 instanceof IntValue) ? ((IntValue) o2).value : ((double[]) o2).length;
        return Integer.compare(len1, len2);
    };

    private static final Comparator<Object> intComparator = (o1, o2) -> {
        int len1 = (o1 instanceof IntValue) ? ((IntValue) o1).value : ((int[]) o1).length;
        int len2 = (o2 instanceof IntValue) ? ((IntValue) o2).value : ((int[]) o2).length;
        return Integer.compare(len1, len2);
    };

    private static final Comparator<Object> objectComparator = (o1, o2) -> {
        int len1 = (o1 instanceof IntValue) ? ((IntValue) o1).value : ((Object[]) o1).length;
        int len2 = (o2 instanceof IntValue) ? ((IntValue) o2).value : ((Object[]) o2).length;
        return Integer.compare(len1, len2);
    };

    private static class IntValue {
        public int value;
    }

    ////////////////////////////////////////////////////////////////////////////

    private static final ThreadLocal<Map> threadLocal = ThreadLocal.withInitial(HashMap::new);

    /**
     * Returns per-thread array pool for given type, or create one if it doesn't exist.
     *
     * @param cls type
     * @return object pool
     */
    @SuppressWarnings("unchecked")
    public static <T> ArrayPool<T> get(Class cls) {
        Map map = threadLocal.get();

        ArrayPool<T> pool = (ArrayPool<T>) map.get(cls);
        if (pool == null) {
            pool = new ArrayPool<T>(cls);
            map.put(cls, pool);
        }

        return pool;
    }

    public static void cleanCurrentThread() {
        threadLocal.remove();
    }

}
