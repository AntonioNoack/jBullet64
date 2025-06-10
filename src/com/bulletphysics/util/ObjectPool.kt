package com.bulletphysics.util

import java.util.function.Supplier

/**
 * Object pool.
 *
 * @author jezek2
 */
class ObjectPool<T>(private val cls: Class<T>) {
    private val list = ObjectArrayList<T>()

    private fun create(): T {
        try {
            return cls.newInstance()
        } catch (e: InstantiationException) {
            throw IllegalStateException(e)
        } catch (e: IllegalAccessException) {
            throw IllegalStateException(e)
        }
    }

    /**
     * Returns instance from pool, or create one if pool is empty.
     *
     * @return instance
     */
    fun get(): T {
        return if (list.isNotEmpty()) {
            list.removeLast()
        } else {
            create()
        }
    }

    /**
     * Release instance into pool.
     *
     * @param obj previously obtained instance from pool
     */
    fun release(obj: T) {
        list.add(obj)
    }

    companion object {
        /** ///////////////////////////////////////////////////////////////////////// */
        private val threadLocal =
            ThreadLocal.withInitial<MutableMap<Class<*>, ObjectPool<*>>>(Supplier { HashMap() })

        /**
         * Returns per-thread object pool for given type, or create one if it doesn't exist.
         *
         * @param cls type
         * @return object pool
         */
        fun <T> get(cls: Class<T>): ObjectPool<T> {
            val map: MutableMap<Class<*>, ObjectPool<*>> = threadLocal.get()

            var pool = map[cls] as? ObjectPool<T>
            if (pool == null) {
                pool = ObjectPool(cls)
                map.put(cls, pool)
            }
            return pool
        }

        @JvmStatic
        fun cleanCurrentThread() {
            threadLocal.remove()
        }
    }
}
