package com.bulletphysics.util;

import java.util.HashMap;
import java.util.Map;

/**
 * Object pool.
 *
 * @author jezek2
 */
public class ObjectPool<T> {

    private final Class<T> cls;
    private final ObjectArrayList<T> list = new ObjectArrayList<T>();

    public ObjectPool(Class<T> cls) {
        this.cls = cls;
    }

    private T create() {
        try {
            return cls.newInstance();
        } catch (InstantiationException | IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

    /**
     * Returns instance from pool, or create one if pool is empty.
     *
     * @return instance
     */
    public T get() {
        if (!list.isEmpty()) {
            return list.remove(list.size() - 1);
        } else {
            return create();
        }
    }

    /**
     * Release instance into pool.
     *
     * @param obj previously obtained instance from pool
     */
    public void release(T obj) {
        list.add(obj);
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static final ThreadLocal<Map<Class<?>, ObjectPool<?>>> threadLocal = ThreadLocal.withInitial(HashMap::new);

    /**
     * Returns per-thread object pool for given type, or create one if it doesn't exist.
     *
     * @param cls type
     * @return object pool
     */
    public static <T> ObjectPool<T> get(Class<T> cls) {
        Map<Class<?>, ObjectPool<?>> map = threadLocal.get();

        @SuppressWarnings("unchecked")
        ObjectPool<T> pool = (ObjectPool<T>) map.get(cls);
        if (pool == null) {
            pool = new ObjectPool<T>(cls);
            map.put(cls, pool);
        }
        return pool;
    }

    public static void cleanCurrentThread() {
        threadLocal.remove();
    }

}
