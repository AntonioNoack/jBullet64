package com.bulletphysics.util;

/**
 * Stack-based object pool for arbitrary objects, returning not supported.
 *
 * @author jezek2
 */
public class ObjectStackList<T> extends StackList<T> {

    private final Class<T> cls;

    public ObjectStackList(Class<T> cls) {
        super();
        this.cls = cls;
    }

    @Override
    protected T create() {
        try {
            return cls.newInstance();
        } catch (InstantiationException | IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
    }
}
