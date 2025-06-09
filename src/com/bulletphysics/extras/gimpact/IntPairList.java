package com.bulletphysics.extras.gimpact;

/**
 * @author jezek2
 */
class IntPairList {

    private long[] content = new long[32];
    private int size = 0;

    public void clear() {
        size = 0;
    }

    public int size() {
        return size;
    }

    public int getFirst(int index) {
        if (index >= size) throw new IndexOutOfBoundsException();
        return (int) (content[index] >>> 32);
    }

    public int getSecond(int index) {
        if (index >= size) throw new IndexOutOfBoundsException();
        return (int) (content[index]);
    }

    private void expand() {
        long[] newArray = new long[content.length << 1];
        System.arraycopy(content, 0, newArray, 0, content.length);
        content = newArray;
    }

    public void pushPair(int index1, int index2) {
        if (size == content.length) {
            expand();
        }
        content[size] = pack(index1, index2);
        size++;
    }

    private static long pack(int index1, int index2) {
        return ((long) index1 << 32) | (((long) index2) & 0xffffffffL);
    }
}
