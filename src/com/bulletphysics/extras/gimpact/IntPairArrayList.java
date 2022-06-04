package com.bulletphysics.extras.gimpact;

/**
 * @author jezek2
 */
class IntPairArrayList {

    private int[] array;
    private int size = 0;

    public IntPairArrayList() {
        array = new int[64];
    }

    public void clear() {
        size = 0;
    }

    public int size() {
        return size;
    }

    public int get(int index) {
        return array[index];
    }

    private void expand() {
        int[] newArray = new int[array.length << 1];
        System.arraycopy(array, 0, newArray, 0, array.length);
        array = newArray;
    }

    public void pushPair(int index1, int index2) {
        if (size == array.length) expand();
        int index = size << 1;
        array[index++] = index1;
        array[index] = index2;
        size++;
    }

}
