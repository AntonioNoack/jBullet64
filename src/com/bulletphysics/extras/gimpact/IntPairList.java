package com.bulletphysics.extras.gimpact;

import static com.bulletphysics.util.Packing.*;

/**
 * @author jezek2
 */
public class IntPairList {

    private long[] content = new long[32];
    public int size = 0;

    public void clear() {
        size = 0;
    }

    public int size() {
        return size;
    }

    public int getFirst(int index) {
        if (index >= size) throw new IndexOutOfBoundsException();
        return getHigh(content[index]);
    }

    public int getSecond(int index) {
        if (index >= size) throw new IndexOutOfBoundsException();
        return getLow(content[index]);
    }

    public long getQuick(int index) {
        return content[index];
    }

    public void setQuick(int index, long value) {
        content[index] = value;
    }

    private void expand() {
        resize(content.length << 1);
    }

    public void resize(int newLength) {
        if (content.length >= newLength && content.length < newLength * 2) {
            return;
        }
        long[] newArray = new long[newLength];
        System.arraycopy(content, 0, newArray, 0, Math.min(content.length, newLength));
        content = newArray;
    }

    public void pushPair(int first, int second) {
        if (size == content.length) {
            expand();
        }
        content[size] = pack(first, second);
        size++;
    }

    public void setPair(int index, int first, int second) {
        content[index] = pack(first, second);
    }
}
