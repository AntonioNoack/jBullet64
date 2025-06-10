package com.bulletphysics.util

/**
 * Stack-based object pool for arbitrary objects, returning not supported.
 *
 * @author jezek2
 */
class ObjectStackList<T>(private val cls: Class<T>) : StackList<T>() {
    override fun create(): T {
        try {
            return cls.newInstance()
        } catch (e: InstantiationException) {
            throw IllegalStateException(e)
        } catch (e: IllegalAccessException) {
            throw IllegalStateException(e)
        }
    }
}
