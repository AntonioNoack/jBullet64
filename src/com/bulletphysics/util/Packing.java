package com.bulletphysics.util;

public class Packing {
    public static long pack(int high, int low) {
        return ((long) high << 32) | (((long) low) & 0xffffffffL);
    }

    public static int getHigh(long value) {
        return (int) (value >>> 32);
    }

    public static int getLow(long value) {
        return (int) value;
    }
}
