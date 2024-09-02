package com.bulletphysics.extras.gimpact;

import com.bulletphysics.extras.gimpact.BoxCollision.AABB;

/**
 * @author jezek2
 */
class BvhTreeNodeArray {

    private int size = 0;

    private double[] bounds = new double[0];
    private int[] indices = new int[0];

    public void clear() {
        size = 0;
    }

    public void resize(int newSize) {
        double[] newBound = new double[newSize * 6];
        int[] newIndices = new int[newSize];

        System.arraycopy(bounds, 0, newBound, 0, size * 6);
        System.arraycopy(indices, 0, newIndices, 0, size);

        bounds = newBound;
        indices = newIndices;

        size = newSize;
    }

    public void set(int dstIdx, BvhTreeNodeArray array, int srcIdx) {
        int dstPos = dstIdx * 6;
        int srcPos = srcIdx * 6;

        double[] aBound = array.bounds, bounds = this.bounds;
        bounds[dstPos] = aBound[srcPos];
        bounds[dstPos + 1] = aBound[srcPos + 1];
        bounds[dstPos + 2] = aBound[srcPos + 2];
        bounds[dstPos + 3] = aBound[srcPos + 3];
        bounds[dstPos + 4] = aBound[srcPos + 4];
        bounds[dstPos + 5] = aBound[srcPos + 5];
        indices[dstIdx] = array.indices[srcIdx];
    }

    public void set(int dstIdx, BvhDataArray array, int srcIdx) {
        int dstPos = dstIdx * 6;
        int srcPos = srcIdx * 6;

        double[] aBound = array.bounds, bounds = this.bounds;
        bounds[dstPos] = aBound[srcPos];
        bounds[dstPos + 1] = aBound[srcPos + 1];
        bounds[dstPos + 2] = aBound[srcPos + 2];
        bounds[dstPos + 3] = aBound[srcPos + 3];
        bounds[dstPos + 4] = aBound[srcPos + 4];
        bounds[dstPos + 5] = aBound[srcPos + 5];
        indices[dstIdx] = array.data[srcIdx];
    }

    public AABB getBounds(int nodeIndex, AABB out) {
        int pos = nodeIndex * 6;
        out.min.set(bounds[pos], bounds[pos + 1], bounds[pos + 2]);
        out.max.set(bounds[pos + 3], bounds[pos + 4], bounds[pos + 5]);
        return out;
    }

    public void setBounds(int nodeIndex, AABB aabb) {
        int pos = nodeIndex * 6;
        bounds[pos] = aabb.min.x;
        bounds[pos + 1] = aabb.min.y;
        bounds[pos + 2] = aabb.min.z;
        bounds[pos + 3] = aabb.max.x;
        bounds[pos + 4] = aabb.max.y;
        bounds[pos + 5] = aabb.max.z;
    }

    public boolean isLeafNode(int nodeIndex) {
        // skipIndex is negative (internal node), triangleIndex >=0 (leaf node)
        return (indices[nodeIndex] >= 0);
    }

    public int getEscapeIndex(int nodeIndex) {
        //btAssert(m_escapeIndexOrDataIndex < 0);
        return -indices[nodeIndex];
    }

    public void setEscapeIndex(int nodeIndex, int index) {
        indices[nodeIndex] = -index;
    }

    public int getDataIndex(int nodeIndex) {
        //btAssert(m_escapeIndexOrDataIndex >= 0);
        return indices[nodeIndex];
    }

    public void setDataIndex(int nodeIndex, int index) {
        indices[nodeIndex] = index;
    }

}
