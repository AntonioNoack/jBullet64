package com.bulletphysics.util;

/**
 * @author jezek2
 */
public class IntArrayList {

    private int[] array = new int[16];
    private int size;

    public void add(int value) {
        if (size == array.length) expand();
        array[size++] = value;
    }

    private void expand() {
        int[] newArray = new int[array.length << 1];
        System.arraycopy(array, 0, newArray, 0, array.length);
        array = newArray;
    }

    public int remove(int index) {
        int old = array[index];
        System.arraycopy(array, index + 1, array, index, size - index - 1);
        size--;
        return old;
    }

    public int get(int index) {
        return array[index];
    }

    public void set(int index, int value) {
        array[index] = value;
    }

    public int size() {
        return size;
    }

    public void clear() {
        size = 0;
    }

}
