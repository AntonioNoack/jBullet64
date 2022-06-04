package com.bulletphysics.util;

import java.util.ArrayList;

/**
 * Stack-based object pool for arbitrary objects, returning not supported.
 *
 * @author jezek2
 */
public class ObjectStackList<T> {

    private final Class<T> cls;

    private final ArrayList<T> list = new ArrayList<>();

    private final int[] stack = new int[512];
    private int stackCount = 0;

    private int pos = 0;

    /**
     * Pushes the stack.
     */
    public final void push() {
		/*if (stackCount == stack.length-1) {
			resizeStack();
		}*/
        stack[stackCount++] = pos;
    }

    /**
     * Pops the stack.
     */
    public final void pop() {
        pos = stack[--stackCount];
    }

    /**
     * Returns instance from stack pool, or create one if not present. The returned
     * instance will be automatically reused when {@link #pop} is called.
     *
     * @return instance
     */
    public T get() {
        if (pos == list.size()) {
            expand();
        }
        return list.get(pos++);
    }

    private void expand() {
        list.add(create());
    }

    public ObjectStackList(Class<T> cls) {
        this.cls = cls;
    }

    /**
     * Creates a new instance of type.
     *
     * @return instance
     */
    protected T create() {
        try {
            return cls.newInstance();
        } catch (InstantiationException | IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }

}
