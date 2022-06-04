package com.bulletphysics.collision.broadphase;

import java.util.AbstractList;
import java.util.Objects;

// it would be nice if we could remove this class...
// only use: HashOverlappingPairCache...
public final class ObjectArrayList<T> extends AbstractList<T> {

    private T[] array;
    private int size;

    public ObjectArrayList() {
        this(16);
    }

    @SuppressWarnings("unchecked")
    public ObjectArrayList(int initialCapacity) {
        array = (T[]) new Object[initialCapacity];
    }

    @Override
    public boolean add(T value) {
        if (size == array.length) expand();
        array[size++] = value;
        return true;
    }

    @Override
    public void add(int index, T value) {
        if (size == array.length) expand();

        int num = size - index;
        if (num > 0) {
            System.arraycopy(array, index, array, index + 1, num);
        }

        array[index] = value;
        size++;
    }

    @Override
    public T remove(int index) {
        T prev = array[index];
        int moved = size - index - 1;
        if (moved > 0) System.arraycopy(array, index + 1, array, index, moved);
        array[size - 1] = null;
        size--;
        return prev;
    }

    @SuppressWarnings("unchecked")
    private void expand() {
        T[] newArray = (T[]) new Object[array.length << 1];
        System.arraycopy(array, 0, newArray, 0, array.length);
        array = newArray;
    }

    public T get(int index) {
        return array[index];
    }

    @Override
    public T set(int index, T value) {
        T old = array[index];
        array[index] = value;
        return old;
    }

    @Override
    public int size() {
        return size;
    }

    public int capacity() {
        return array.length;
    }

    @Override
    public void clear() {
        size = 0;
    }

    @Override
    public int indexOf(Object o) {
        int _size = size;
        T[] _array = array;
        for (int i = 0; i < _size; i++) {
            if (Objects.equals(o, _array[i])) {
                return i;
            }
        }
        return -1;
    }


}