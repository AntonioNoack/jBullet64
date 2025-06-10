package com.bulletphysics.extras.gimpact;

import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
class BvhTree {

    protected int numNodes = 0;
    protected BvhTreeNodeArray nodes = new BvhTreeNodeArray();

    protected int calcSplittingAxis(BvhDataArray primitiveBoxes, int startIndex, int endIndex) {
        Vector3d means = Stack.newVec(0.0);
        Vector3d variance = Stack.newVec(0.0);

        int numIndices = endIndex - startIndex;

        Vector3d center = Stack.newVec();
        Vector3d diff2 = Stack.newVec();

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        for (int i = startIndex; i < endIndex; i++) {
            primitiveBoxes.getBoundsMax(i, tmp1);
            primitiveBoxes.getBoundsMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);
            means.add(center);
        }
        means.scale(1.0 / (double) numIndices);

        for (int i = startIndex; i < endIndex; i++) {
            primitiveBoxes.getBoundsMax(i, tmp1);
            primitiveBoxes.getBoundsMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);
            diff2.sub(center, means);
            VectorUtil.mul(diff2, diff2, diff2);
            variance.add(diff2);
        }
        variance.scale(1.0 / (double) (numIndices - 1));

        return VectorUtil.maxAxis(variance);
    }

    protected int sortAndCalcSplittingIndex(BvhDataArray primitiveBoxes, int startIndex, int endIndex, int splitAxis) {
        int splitIndex = startIndex;
        int numIndices = endIndex - startIndex;

        Vector3d means = Stack.newVec(0.0);

        Vector3d center = Stack.newVec();

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        for (int i = startIndex; i < endIndex; i++) {
            primitiveBoxes.getBoundsMax(i, tmp1);
            primitiveBoxes.getBoundsMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);
            means.add(center);
        }
        means.scale(1.0 / numIndices);

        // average of centers
        double splitValue = VectorUtil.getCoord(means, splitAxis);

        // sort leafNodes so all values larger than splitValue comes first, and smaller values start from 'splitIndex'.
        for (int i = startIndex; i < endIndex; i++) {
            primitiveBoxes.getBoundsMax(i, tmp1);
            primitiveBoxes.getBoundsMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);

            if (VectorUtil.getCoord(center, splitAxis) > splitValue) {
                // swap
                primitiveBoxes.swap(i, splitIndex);
                //swapLeafNodes(i,splitIndex);
                splitIndex++;
            }
        }

        // if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
        // otherwise the tree-building might fail due to stack-overflows in certain cases.
        // unbalanced1 is unsafe: it can cause stack overflows
        //bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

        // unbalanced2 should work too: always use center (perfect balanced trees)
        //bool unbalanced2 = true;

        // this should be safe too:
        int rangeBalancedIndices = numIndices / 3;
        boolean unbalanced = ((splitIndex <= (startIndex + rangeBalancedIndices)) || (splitIndex >= (endIndex - 1 - rangeBalancedIndices)));

        if (unbalanced) {
            splitIndex = startIndex + (numIndices >> 1);
        }

        boolean unbal = (splitIndex == startIndex) || (splitIndex == (endIndex));
        assert (!unbal);

        return splitIndex;
    }

    protected void buildSubTree(BvhDataArray primitiveBoxes, int startIndex, int endIndex) {
        int curIndex = numNodes;
        numNodes++;

        assert ((endIndex - startIndex) > 0);

        if ((endIndex - startIndex) == 1) {
            // We have a leaf node
            //setNodeBound(curIndex,primitive_boxes[startIndex].m_bound);
            //m_node_array[curIndex].setDataIndex(primitive_boxes[startIndex].m_data);
            nodes.set(curIndex, primitiveBoxes, startIndex);

            return;
        }
        // calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

        // split axis
        int splitIndex = calcSplittingAxis(primitiveBoxes, startIndex, endIndex);

        splitIndex = sortAndCalcSplittingIndex(primitiveBoxes, startIndex, endIndex, splitIndex);

        //calc this node bounding box

        AABB nodeBound = new AABB();
        AABB tmpAABB = new AABB();

        nodeBound.invalidate();

        for (int i = startIndex; i < endIndex; i++) {
            primitiveBoxes.getBounds(i, tmpAABB);
            nodeBound.merge(tmpAABB);
        }

        setNodeBound(curIndex, nodeBound);

        // build left branch
        buildSubTree(primitiveBoxes, startIndex, splitIndex);

        // build right branch
        buildSubTree(primitiveBoxes, splitIndex, endIndex);

        nodes.setEscapeIndex(curIndex, numNodes - curIndex);
    }

    public void buildTree(BvhDataArray primitiveBoxes) {
        // initialize node count to 0
        numNodes = 0;
        // allocate nodes
        nodes.resize(primitiveBoxes.size() * 2);

        buildSubTree(primitiveBoxes, 0, primitiveBoxes.size());
    }

    public void clearNodes() {
        nodes.clear();
        numNodes = 0;
    }

    public int getNodeCount() {
        return numNodes;
    }

    /**
     * Tells if the node is a leaf.
     */
    public boolean isLeafNode(int nodeIndex) {
        return nodes.isLeafNode(nodeIndex);
    }

    public int getNodeData(int nodeIndex) {
        return nodes.getDataIndex(nodeIndex);
    }

    public void getNodeBound(int nodeIndex, AABB bound) {
        nodes.getBounds(nodeIndex, bound);
    }

    public void setNodeBound(int nodeIndex, AABB bound) {
        nodes.setBounds(nodeIndex, bound);
    }

    public int getLeftNode(int nodeIndex) {
        return nodeIndex + 1;
    }

    public int getRightNode(int nodeIndex) {
        if (nodes.isLeafNode(nodeIndex + 1)) {
            return nodeIndex + 2;
        }
        return nodeIndex + 1 + nodes.getEscapeIndex(nodeIndex + 1);
    }

    public int getEscapeNodeIndex(int nodeIndex) {
        return nodes.getEscapeIndex(nodeIndex);
    }

    public BvhTreeNodeArray getNodePointer() {
        return nodes;
    }

}
