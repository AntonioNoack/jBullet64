package com.bulletphysics.extras.gimpact;

import com.bulletphysics.extras.gimpact.BoxCollision.AABB;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
class BvhDataArray {

    private int size = 0;

    double[] bounds = new double[0];
    int[] data = new int[0];

    public int size() {
        return size;
    }

    public void resize(int newSize) {
        double[] newBound = new double[newSize * 6];
        int[] newData = new int[newSize];

        System.arraycopy(bounds, 0, newBound, 0, size * 6);
        System.arraycopy(data, 0, newData, 0, size);

        bounds = newBound;
        data = newData;

        size = newSize;
    }

    public void swap(int idx1, int idx2) {
        int pos1 = idx1 * 6;
        int pos2 = idx2 * 6;

        double b0 = bounds[pos1];
        double b1 = bounds[pos1 + 1];
        double b2 = bounds[pos1 + 2];
        double b3 = bounds[pos1 + 3];
        double b4 = bounds[pos1 + 4];
        double b5 = bounds[pos1 + 5];
        int d = data[idx1];

        bounds[pos1] = bounds[pos2];
        bounds[pos1 + 1] = bounds[pos2 + 1];
        bounds[pos1 + 2] = bounds[pos2 + 2];
        bounds[pos1 + 3] = bounds[pos2 + 3];
        bounds[pos1 + 4] = bounds[pos2 + 4];
        bounds[pos1 + 5] = bounds[pos2 + 5];
        data[idx1] = data[idx2];

        bounds[pos2] = b0;
        bounds[pos2 + 1] = b1;
        bounds[pos2 + 2] = b2;
        bounds[pos2 + 3] = b3;
        bounds[pos2 + 4] = b4;
        bounds[pos2 + 5] = b5;
        data[idx2] = d;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AABB getBounds(int idx, AABB out) {
        int pos = idx * 6;
        out.min.set(bounds[pos], bounds[pos + 1], bounds[pos + 2]);
        out.max.set(bounds[pos + 3], bounds[pos + 4], bounds[pos + 5]);
        return out;
    }

    @SuppressWarnings("UnusedReturnValue")
    public Vector3d getBoundsMin(int idx, Vector3d out) {
        int pos = idx * 6;
        out.set(bounds[pos], bounds[pos + 1], bounds[pos + 2]);
        return out;
    }

    @SuppressWarnings("UnusedReturnValue")
    public Vector3d getBoundsMax(int idx, Vector3d out) {
        int pos = idx * 6;
        out.set(bounds[pos + 3], bounds[pos + 4], bounds[pos + 5]);
        return out;
    }

    public void setBounds(int idx, AABB aabb) {
        int pos = idx * 6;
        bounds[pos] = aabb.min.x;
        bounds[pos + 1] = aabb.min.y;
        bounds[pos + 2] = aabb.min.z;
        bounds[pos + 3] = aabb.max.x;
        bounds[pos + 4] = aabb.max.y;
        bounds[pos + 5] = aabb.max.z;
    }

    public int getData(int idx) {
        return data[idx];
    }

    public void setData(int idx, int value) {
        data[idx] = value;
    }

}
