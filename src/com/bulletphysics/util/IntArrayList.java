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

    public void setSize(int newSize) {
        if (array.length < newSize) {
            int newSize2 = array.length;
            while (newSize2 < newSize) {
                newSize2 <<= 1;
            }
            int[] newArray = new int[newSize2];
            System.arraycopy(array, 0, newArray, 0, array.length);
            array = newArray;
        }
        for (int i = size; i < newSize; i++) {
            array[i] = 0;
        }
        size = newSize;
    }

    public void pushPair(int a, int b) {
        add(a);
        add(b);
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
