/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
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

    protected int _calc_splitting_axis(BvhDataArray primitive_boxes, int startIndex, int endIndex) {
        Vector3d means = Stack.newVec(0.0);
        Vector3d variance = Stack.newVec(0.0);

        int numIndices = endIndex - startIndex;

        Vector3d center = Stack.newVec();
        Vector3d diff2 = Stack.newVec();

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        for (int i = startIndex; i < endIndex; i++) {
            primitive_boxes.getBoundMax(i, tmp1);
            primitive_boxes.getBoundMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);
            means.add(center);
        }
        means.scale(1.0 / (double) numIndices);

        for (int i = startIndex; i < endIndex; i++) {
            primitive_boxes.getBoundMax(i, tmp1);
            primitive_boxes.getBoundMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);
            diff2.sub(center, means);
            VectorUtil.mul(diff2, diff2, diff2);
            variance.add(diff2);
        }
        variance.scale(1.0 / (double) (numIndices - 1));

        Stack.subVec(6);
        
        return VectorUtil.maxAxis(variance);
    }

    protected int _sort_and_calc_splitting_index(BvhDataArray primitive_boxes, int startIndex, int endIndex, int splitAxis) {
        int splitIndex = startIndex;
        int numIndices = endIndex - startIndex;

        // average of centers
        double splitValue;

        Vector3d means = Stack.newVec(0.0);

        Vector3d center = Stack.newVec();

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        for (int i = startIndex; i < endIndex; i++) {
            primitive_boxes.getBoundMax(i, tmp1);
            primitive_boxes.getBoundMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);
            means.add(center);
        }
        means.scale(1.0 / numIndices);

        splitValue = VectorUtil.getCoord(means, splitAxis);

        // sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
        for (int i = startIndex; i < endIndex; i++) {
            primitive_boxes.getBoundMax(i, tmp1);
            primitive_boxes.getBoundMin(i, tmp2);
            center.add(tmp1, tmp2);
            center.scale(0.5);

            if (VectorUtil.getCoord(center, splitAxis) > splitValue) {
                // swap
                primitive_boxes.swap(i, splitIndex);
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
        
        Stack.subVec(4);

        return splitIndex;
    }

    protected void buildSubTree(BvhDataArray primitive_boxes, int startIndex, int endIndex) {
        int curIndex = numNodes;
        numNodes++;

        assert ((endIndex - startIndex) > 0);

        if ((endIndex - startIndex) == 1) {
            // We have a leaf node
            //setNodeBound(curIndex,primitive_boxes[startIndex].m_bound);
            //m_node_array[curIndex].setDataIndex(primitive_boxes[startIndex].m_data);
            nodes.set(curIndex, primitive_boxes, startIndex);

            return;
        }
        // calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

        // split axis
        int splitIndex = _calc_splitting_axis(primitive_boxes, startIndex, endIndex);

        splitIndex = _sort_and_calc_splitting_index(primitive_boxes, startIndex, endIndex, splitIndex);

        //calc this node bounding box

        AABB nodeBound = new AABB();
        AABB tmpAABB = new AABB();

        nodeBound.invalidate();

        for (int i = startIndex; i < endIndex; i++) {
            primitive_boxes.getBound(i, tmpAABB);
            nodeBound.merge(tmpAABB);
        }

        setNodeBound(curIndex, nodeBound);

        // build left branch
        buildSubTree(primitive_boxes, startIndex, splitIndex);

        // build right branch
        buildSubTree(primitive_boxes, splitIndex, endIndex);

        nodes.setEscapeIndex(curIndex, numNodes - curIndex);
    }

    public void buildTree(BvhDataArray primitive_boxes) {
        // initialize node count to 0
        numNodes = 0;
        // allocate nodes
        nodes.resize(primitive_boxes.size() * 2);

        buildSubTree(primitive_boxes, 0, primitive_boxes.size());
    }

    public void clear() {
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
        nodes.getBound(nodeIndex, bound);
    }

    public void setNodeBound(int nodeIndex, AABB bound) {
        nodes.setBound(nodeIndex, bound);
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

    public BvhTreeNodeArray get_node_pointer() {
        return nodes;
    }

}
