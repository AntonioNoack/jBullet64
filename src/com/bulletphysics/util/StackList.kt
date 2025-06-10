package com.bulletphysics.util

/**
 * Stack-based object pool, see the example for usage. You must use the [.returning]
 * method for returning stack-allocated instance.
 *
 *
 *
 * Example code:
 *
 * <pre>
 * StackList&lt;Vector3d&gt; vectors;
 * ...
 *
 * vectors.push();
 * try {
 * Vector3d vec = vectors.get();
 * ...
 * return vectors.returning(vec);
 * }
 * finally {
 * vectors.pop();
 * }
</pre> *
 *
 * @author jezek2
 */
abstract class StackList<T> protected constructor() {
    private val list = ObjectArrayList<T>()

    private val stack = IntArray(512)
    private var stackCount = 0

    private var pos = 0

    /**
     * Pushes the stack.
     */
    fun push() {
        stack[stackCount++] = pos
    }

    /**
     * Pops the stack.
     */
    fun pop() {
        pos = stack[--stackCount]
    }

    /**
     * Returns instance from stack pool, or create one if not present. The returned
     * instance will be automatically reused when [.pop] is called.
     *
     * @return instance
     */
    fun get(): T {
        if (pos == list.size) {
            expand()
        }
        return list.getQuick(pos++)
    }

    /**
     * Creates a new instance of type.
     *
     * @return instance
     */
    protected abstract fun create(): T

    private fun expand() {
        list.add(create())
    }
}
