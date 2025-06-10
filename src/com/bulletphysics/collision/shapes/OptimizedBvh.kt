package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import java.io.Serializable;

import static java.lang.Math.max;
import static java.lang.Math.min;

// JAVA NOTE: OptimizedBvh still from 2.66, update it for 2.70b1

/**
 * OptimizedBvh store an AABB tree that can be quickly traversed on CPU (and SPU, GPU in future).
 *
 * @author jezek2
 */
public class OptimizedBvh implements Serializable {

    private static final boolean DEBUG_TREE_BUILDING = false;
    private static int gStackDepth = 0;
    private static int gMaxStackDepth = 0;

    private static int maxIterations = 0;

    // Note: currently we have 16 bytes per quantized node
    public static final int MAX_SUBTREE_SIZE_IN_BYTES = 2048;

    // 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
    // actually) triangles each (since the sign bit is reserved
    public static final int MAX_NUM_PARTS_IN_BITS = 10;

    /// /////////////////////////////////////////////////////////////////////////

    private final ObjectArrayList<OptimizedBvhNode> leafNodes = new ObjectArrayList<OptimizedBvhNode>();
    private final ObjectArrayList<OptimizedBvhNode> contiguousNodes = new ObjectArrayList<OptimizedBvhNode>();

    private final QuantizedBvhNodes quantizedLeafNodes = new QuantizedBvhNodes();
    private final QuantizedBvhNodes quantizedContiguousNodes = new QuantizedBvhNodes();

    private int curNodeIndex;

    // quantization data
    private boolean useQuantization;
    private final Vector3d bvhAabbMin = new Vector3d();
    private final Vector3d bvhAabbMax = new Vector3d();
    private final Vector3d bvhQuantization = new Vector3d();

    protected TraversalMode traversalMode = TraversalMode.STACKLESS;
    protected final ObjectArrayList<BvhSubtreeInfo> SubtreeHeaders = new ObjectArrayList<BvhSubtreeInfo>();
    // This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray
    protected int subtreeHeaderCount;

    // two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
    // this might be refactored into a virtual, it is usually not calculated at run-time
    public void setInternalNodeAabbMin(int nodeIndex, Vector3d aabbMin) {
        if (useQuantization) {
            quantizedContiguousNodes.setQuantizedAabbMin(nodeIndex, quantizeWithClamp(aabbMin));
        } else {
            contiguousNodes.getQuick(nodeIndex).aabbMinOrg.set(aabbMin);
        }
    }

    public void setInternalNodeAabbMax(int nodeIndex, Vector3d aabbMax) {
        if (useQuantization) {
            quantizedContiguousNodes.setQuantizedAabbMax(nodeIndex, quantizeWithClamp(aabbMax));
        } else {
            contiguousNodes.getQuick(nodeIndex).aabbMaxOrg.set(aabbMax);
        }
    }

    public Vector3d getAabbMin(int nodeIndex) {
        if (useQuantization) {
            Vector3d tmp = new Vector3d();
            unQuantize(tmp, quantizedLeafNodes.getQuantizedAabbMin(nodeIndex));
            return tmp;
        }

        // non-quantized
        return leafNodes.getQuick(nodeIndex).aabbMinOrg;
    }

    public Vector3d getAabbMax(int nodeIndex) {
        if (useQuantization) {
            Vector3d tmp = new Vector3d();
            unQuantize(tmp, quantizedLeafNodes.getQuantizedAabbMax(nodeIndex));
            return tmp;
        }

        // non-quantized
        return leafNodes.getQuick(nodeIndex).aabbMaxOrg;
    }

    public void setQuantizationValues(Vector3d aabbMin, Vector3d aabbMax) {
        setQuantizationValues(aabbMin, aabbMax, 1.0);
    }

    public void setQuantizationValues(Vector3d aabbMin, Vector3d aabbMax, double quantizationMargin) {
        // enlarge the AABB to avoid division by zero when initializing the quantization values
        Vector3d clampValue = Stack.newVec();
        clampValue.set(quantizationMargin, quantizationMargin, quantizationMargin);
        bvhAabbMin.sub(aabbMin, clampValue);
        bvhAabbMax.add(aabbMax, clampValue);
        Vector3d aabbSize = Stack.newVec();
        aabbSize.sub(bvhAabbMax, bvhAabbMin);
        bvhQuantization.set(65535f, 65535f, 65535f);
        VectorUtil.div(bvhQuantization, bvhQuantization, aabbSize);
        Stack.subVec(2);
    }

    public void setInternalNodeEscapeIndex(int nodeIndex, int escapeIndex) {
        if (useQuantization) {
            quantizedContiguousNodes.setEscapeIndexOrTriangleIndex(nodeIndex, -escapeIndex);
        } else {
            contiguousNodes.getQuick(nodeIndex).escapeIndex = escapeIndex;
        }
    }

    public void mergeInternalNodeAabb(int nodeIndex, Vector3d newAabbMin, Vector3d newAabbMax) {
        if (useQuantization) {
            long quantizedAabbMin;
            long quantizedAabbMax;

            quantizedAabbMin = quantizeWithClamp(newAabbMin);
            quantizedAabbMax = quantizeWithClamp(newAabbMax);
            for (int i = 0; i < 3; i++) {
                if (quantizedContiguousNodes.getQuantizedAabbMin(nodeIndex, i) > QuantizedBvhNodes.getCoord(quantizedAabbMin, i)) {
                    quantizedContiguousNodes.setQuantizedAabbMin(nodeIndex, i, QuantizedBvhNodes.getCoord(quantizedAabbMin, i));
                }

                if (quantizedContiguousNodes.getQuantizedAabbMax(nodeIndex, i) < QuantizedBvhNodes.getCoord(quantizedAabbMax, i)) {
                    quantizedContiguousNodes.setQuantizedAabbMax(nodeIndex, i, QuantizedBvhNodes.getCoord(quantizedAabbMax, i));
                }
            }
        } else {
            // non-quantized
            VectorUtil.setMin(contiguousNodes.getQuick(nodeIndex).aabbMinOrg, newAabbMin);
            VectorUtil.setMax(contiguousNodes.getQuick(nodeIndex).aabbMaxOrg, newAabbMax);
        }
    }

    public void swapLeafNodes(int i, int splitIndex) {
        if (useQuantization) {
            quantizedLeafNodes.swap(i, splitIndex);
        } else {
            // JAVA NOTE: changing reference instead of copy
            OptimizedBvhNode tmp = leafNodes.getQuick(i);
            leafNodes.setQuick(i, leafNodes.getQuick(splitIndex));
            leafNodes.setQuick(splitIndex, tmp);
        }
    }

    public void assignInternalNodeFromLeafNode(int internalNode, int leafNodeIndex) {
        if (useQuantization) {
            quantizedContiguousNodes.set(internalNode, quantizedLeafNodes, leafNodeIndex);
        } else {
            contiguousNodes.getQuick(internalNode).set(leafNodes.getQuick(leafNodeIndex));
        }
    }

    private static class NodeTriangleCallback implements InternalTriangleIndexCallback {
        public ObjectArrayList<OptimizedBvhNode> triangleNodes;

        public NodeTriangleCallback(ObjectArrayList<OptimizedBvhNode> triangleNodes) {
            this.triangleNodes = triangleNodes;
        }

        private final Vector3d aabbMin = new Vector3d(), aabbMax = new Vector3d();

        public void internalProcessTriangleIndex(Vector3d[] triangle, int partId, int triangleIndex) {
            OptimizedBvhNode node = new OptimizedBvhNode();
            aabbMin.set(1e308, 1e308, 1e308);
            aabbMax.set(-1e308, -1e308, -1e308);
            VectorUtil.setMin(aabbMin, triangle[0]);
            VectorUtil.setMax(aabbMax, triangle[0]);
            VectorUtil.setMin(aabbMin, triangle[1]);
            VectorUtil.setMax(aabbMax, triangle[1]);
            VectorUtil.setMin(aabbMin, triangle[2]);
            VectorUtil.setMax(aabbMax, triangle[2]);

            // with quantization?
            node.aabbMinOrg.set(aabbMin);
            node.aabbMaxOrg.set(aabbMax);

            node.escapeIndex = -1;

            // for child nodes
            node.subPart = partId;
            node.triangleIndex = triangleIndex;
            triangleNodes.add(node);
        }
    }

    private static class QuantizedNodeTriangleCallback implements InternalTriangleIndexCallback {

        public QuantizedBvhNodes triangleNodes;
        public OptimizedBvh optimizedTree; // for quantization

        public QuantizedNodeTriangleCallback(QuantizedBvhNodes triangleNodes, OptimizedBvh tree) {
            this.triangleNodes = triangleNodes;
            this.optimizedTree = tree;
        }

        public void internalProcessTriangleIndex(Vector3d[] triangle, int partId, int triangleIndex) {
            // The partId and triangle index must fit in the same (positive) integer
            assert (partId < (1 << MAX_NUM_PARTS_IN_BITS));
            assert (triangleIndex < (1 << (31 - MAX_NUM_PARTS_IN_BITS)));
            // negative indices are reserved for escapeIndex
            assert (triangleIndex >= 0);

            int nodeId = triangleNodes.add();
            Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
            aabbMin.set(1e308, 1e308, 1e308);
            aabbMax.set(-1e308, -1e308, -1e308);
            VectorUtil.setMin(aabbMin, triangle[0]);
            VectorUtil.setMax(aabbMax, triangle[0]);
            VectorUtil.setMin(aabbMin, triangle[1]);
            VectorUtil.setMax(aabbMax, triangle[1]);
            VectorUtil.setMin(aabbMin, triangle[2]);
            VectorUtil.setMax(aabbMax, triangle[2]);

            // PCK: add these checks for zero dimensions of aabb
            final double MIN_AABB_DIMENSION = 0.002f;
            final double MIN_AABB_HALF_DIMENSION = 0.001f;
            if (aabbMax.x - aabbMin.x < MIN_AABB_DIMENSION) {
                aabbMax.x = (aabbMax.x + MIN_AABB_HALF_DIMENSION);
                aabbMin.x = (aabbMin.x - MIN_AABB_HALF_DIMENSION);
            }
            if (aabbMax.y - aabbMin.y < MIN_AABB_DIMENSION) {
                aabbMax.y = (aabbMax.y + MIN_AABB_HALF_DIMENSION);
                aabbMin.y = (aabbMin.y - MIN_AABB_HALF_DIMENSION);
            }
            if (aabbMax.z - aabbMin.z < MIN_AABB_DIMENSION) {
                aabbMax.z = (aabbMax.z + MIN_AABB_HALF_DIMENSION);
                aabbMin.z = (aabbMin.z - MIN_AABB_HALF_DIMENSION);
            }

            triangleNodes.setQuantizedAabbMin(nodeId, optimizedTree.quantizeWithClamp(aabbMin));
            triangleNodes.setQuantizedAabbMax(nodeId, optimizedTree.quantizeWithClamp(aabbMax));

            triangleNodes.setEscapeIndexOrTriangleIndex(nodeId, (partId << (31 - MAX_NUM_PARTS_IN_BITS)) | triangleIndex);
            Stack.subVec(2);
        }
    }

    public void build(StridingMeshInterface triangles, boolean useQuantizedAabbCompression, Vector3d _aabbMin, Vector3d _aabbMax) {
        this.useQuantization = useQuantizedAabbCompression;

        // NodeArray	triangleNodes;

        int numLeafNodes;

        if (useQuantization) {
            // initialize quantization values
            setQuantizationValues(_aabbMin, _aabbMax);

            QuantizedNodeTriangleCallback callback = new QuantizedNodeTriangleCallback(quantizedLeafNodes, this);

            triangles.internalProcessAllTriangles(callback, bvhAabbMin, bvhAabbMax);

            // now we have an array of leafnodes in m_leafNodes
            numLeafNodes = quantizedLeafNodes.size();

            quantizedContiguousNodes.resize(2 * numLeafNodes);
        } else {
            NodeTriangleCallback callback = new NodeTriangleCallback(leafNodes);

            Vector3d aabbMin = Stack.newVec();
            Vector3d aabbMax = Stack.newVec();
            aabbMin.set(-1e308, -1e308, -1e308);
            aabbMax.set(1e308, 1e308, 1e308);

            triangles.internalProcessAllTriangles(callback, aabbMin, aabbMax);
            Stack.subVec(2);

            // now we have an array of leafnodes in m_leafNodes
            numLeafNodes = leafNodes.size();

            // TODO: check
            //contiguousNodes.resize(2*numLeafNodes);
            MiscUtil.resize(contiguousNodes, 2 * numLeafNodes, OptimizedBvhNode.class);
        }

        curNodeIndex = 0;

        buildTree(0, numLeafNodes);

        //  if the entire tree is small then subtree size, we need to create a header info for the tree
        if (useQuantization && SubtreeHeaders.isEmpty()) {
            BvhSubtreeInfo subtree = new BvhSubtreeInfo();
            SubtreeHeaders.add(subtree);

            subtree.setAabbFromQuantizeNode(quantizedContiguousNodes, 0);
            subtree.rootNodeIndex = 0;
            subtree.subtreeSize = quantizedContiguousNodes.isLeafNode(0) ? 1 : quantizedContiguousNodes.getEscapeIndex(0);
        }

        // PCK: update the copy of the size
        subtreeHeaderCount = SubtreeHeaders.size();

        // PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
        quantizedLeafNodes.clear();
        leafNodes.clear();
    }

    public void refit(StridingMeshInterface meshInterface) {
        if (useQuantization) {
            // calculate new aabb
            Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
            meshInterface.calculateAabbBruteForce(aabbMin, aabbMax);

            setQuantizationValues(aabbMin, aabbMax);

            updateBvhNodes(meshInterface, 0, curNodeIndex);

            // now update all subtree headers

            int i;
            for (i = 0; i < SubtreeHeaders.size(); i++) {
                BvhSubtreeInfo subtree = SubtreeHeaders.getQuick(i);
                subtree.setAabbFromQuantizeNode(quantizedContiguousNodes, subtree.rootNodeIndex);
            }

        } else {
            // JAVA NOTE: added for testing, it's too slow for practical use
            build(meshInterface, false, null, null);
        }
    }

    public void refitPartial(StridingMeshInterface meshInterface, Vector3d aabbMin, Vector3d aabbMax) {
        throw new UnsupportedOperationException();
//		// incrementally initialize quantization values
//		assert (useQuantization);
//
//		btAssert(aabbMin.getX() > m_bvhAabbMin.getX());
//		btAssert(aabbMin.getY() > m_bvhAabbMin.getY());
//		btAssert(aabbMin.getZ() > m_bvhAabbMin.getZ());
//
//		btAssert(aabbMax.getX() < m_bvhAabbMax.getX());
//		btAssert(aabbMax.getY() < m_bvhAabbMax.getY());
//		btAssert(aabbMax.getZ() < m_bvhAabbMax.getZ());
//
//		///we should update all quantization values, using updateBvhNodes(meshInterface);
//		///but we only update chunks that overlap the given aabb
//
//		unsigned short	quantizedQueryAabbMin[3];
//		unsigned short	quantizedQueryAabbMax[3];
//
//		quantizeWithClamp(&quantizedQueryAabbMin[0],aabbMin);
//		quantizeWithClamp(&quantizedQueryAabbMax[0],aabbMax);
//
//		int i;
//		for (i=0;i<this->m_SubtreeHeaders.size();i++)
//		{
//			btBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];
//
//			//PCK: unsigned instead of bool
//			unsigned overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
//			if (overlap != 0)
//			{
//				updateBvhNodes(meshInterface,subtree.m_rootNodeIndex,subtree.m_rootNodeIndex+subtree.m_subtreeSize,i);
//
//				subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes[subtree.m_rootNodeIndex]);
//			}
//		}
    }

    public void updateBvhNodes(StridingMeshInterface meshInterface, int firstNode, int endNode) {
        assert (useQuantization);

        int curNodeSubPart = -1;

        Vector3d[] triangleVerts = new Vector3d[]{Stack.newVec(), Stack.newVec(), Stack.newVec()};
        Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
        Vector3d meshScaling = meshInterface.getScaling(Stack.newVec());

        VertexData data = null;

        for (int i = endNode - 1; i >= firstNode; i--) {
            QuantizedBvhNodes curNodes = quantizedContiguousNodes;

            if (curNodes.isLeafNode(i)) {
                // recalc aabb from triangle data
                int nodeSubPart = curNodes.getPartId(i);
                int nodeTriangleIndex = curNodes.getTriangleIndex(i);
                if (nodeSubPart != curNodeSubPart) {
                    data = meshInterface.getLockedReadOnlyVertexIndexBase(nodeSubPart);
                }

                assert data != null;
                data.getTriangle(nodeTriangleIndex * 3, meshScaling, triangleVerts);

                aabbMin.set(1e308, 1e308, 1e308);
                aabbMax.set(-1e308, -1e308, -1e308);
                VectorUtil.setMin(aabbMin, triangleVerts[0]);
                VectorUtil.setMax(aabbMax, triangleVerts[0]);
                VectorUtil.setMin(aabbMin, triangleVerts[1]);
                VectorUtil.setMax(aabbMax, triangleVerts[1]);
                VectorUtil.setMin(aabbMin, triangleVerts[2]);
                VectorUtil.setMax(aabbMax, triangleVerts[2]);

                curNodes.setQuantizedAabbMin(i, quantizeWithClamp(aabbMin));
                curNodes.setQuantizedAabbMax(i, quantizeWithClamp(aabbMax));
            } else {
                // combine aabb from both children

                //quantizedContiguousNodes
                int leftChildNodeId = i + 1;

                int rightChildNodeId = quantizedContiguousNodes.isLeafNode(leftChildNodeId) ? i + 2 : i + 1 + quantizedContiguousNodes.getEscapeIndex(leftChildNodeId);

                for (int i2 = 0; i2 < 3; i2++) {
                    curNodes.setQuantizedAabbMin(i, i2, quantizedContiguousNodes.getQuantizedAabbMin(leftChildNodeId, i2));
                    if (curNodes.getQuantizedAabbMin(i, i2) > quantizedContiguousNodes.getQuantizedAabbMin(rightChildNodeId, i2)) {
                        curNodes.setQuantizedAabbMin(i, i2, quantizedContiguousNodes.getQuantizedAabbMin(rightChildNodeId, i2));
                    }

                    curNodes.setQuantizedAabbMax(i, i2, quantizedContiguousNodes.getQuantizedAabbMax(leftChildNodeId, i2));
                    if (curNodes.getQuantizedAabbMax(i, i2) < quantizedContiguousNodes.getQuantizedAabbMax(rightChildNodeId, i2)) {
                        curNodes.setQuantizedAabbMax(i, i2, quantizedContiguousNodes.getQuantizedAabbMax(rightChildNodeId, i2));
                    }
                }
            }
        }
        Stack.subVec(6);
    }

    protected void buildTree(int startIndex, int endIndex) {
        //#ifdef DEBUG_TREE_BUILDING
        if (DEBUG_TREE_BUILDING) {
            gStackDepth++;
            if (gStackDepth > gMaxStackDepth) {
                gMaxStackDepth = gStackDepth;
            }
        }
        //#endif //DEBUG_TREE_BUILDING

        int splitAxis, splitIndex, i;
        int numIndices = endIndex - startIndex;
        int curIndex = curNodeIndex;

        assert (numIndices > 0);

        if (numIndices == 1) {
            //#ifdef DEBUG_TREE_BUILDING
            if (DEBUG_TREE_BUILDING) {
                gStackDepth--;
            }
            //#endif //DEBUG_TREE_BUILDING

            assignInternalNodeFromLeafNode(curNodeIndex, startIndex);

            curNodeIndex++;
            return;
        }
        // calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

        splitAxis = calcSplittingAxis(startIndex, endIndex);
        splitIndex = sortAndCalcSplittingIndex(startIndex, endIndex, splitAxis);

        int internalNodeIndex = curNodeIndex;

        Vector3d tmp = Stack.newVec();
        tmp.set(-1e308, -1e308, -1e308);
        setInternalNodeAabbMax(curNodeIndex, tmp);
        tmp.set(1e308, 1e308, 1e308);
        setInternalNodeAabbMin(curNodeIndex, tmp);
        Stack.subVec(1);

        for (i = startIndex; i < endIndex; i++) {
            mergeInternalNodeAabb(curNodeIndex, getAabbMin(i), getAabbMax(i));
        }

        curNodeIndex++;

        //internalNode->m_escapeIndex;

        int leftChildNodexIndex = curNodeIndex;

        //build left child tree
        buildTree(startIndex, splitIndex);

        int rightChildNodexIndex = curNodeIndex;
        // build right child tree
        buildTree(splitIndex, endIndex);

        //#ifdef DEBUG_TREE_BUILDING
        if (DEBUG_TREE_BUILDING) {
            gStackDepth--;
        }
        //#endif //DEBUG_TREE_BUILDING

        int escapeIndex = curNodeIndex - curIndex;

        if (useQuantization) {
            // escapeIndex is the number of nodes of this subtree
            int sizeQuantizedNode = QuantizedBvhNodes.getNodeSize();
            int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
            if (treeSizeInBytes > MAX_SUBTREE_SIZE_IN_BYTES) {
                updateSubtreeHeaders(leftChildNodexIndex, rightChildNodexIndex);
            }
        }

        setInternalNodeEscapeIndex(internalNodeIndex, escapeIndex);
    }

    protected boolean testQuantizedAabbAgainstQuantizedAabb(long aabbMin1, long aabbMax1, long aabbMin2, long aabbMax2) {
        int aabbMin1_0 = QuantizedBvhNodes.getCoord(aabbMin1, 0);
        int aabbMin1_1 = QuantizedBvhNodes.getCoord(aabbMin1, 1);
        int aabbMin1_2 = QuantizedBvhNodes.getCoord(aabbMin1, 2);

        int aabbMax1_0 = QuantizedBvhNodes.getCoord(aabbMax1, 0);
        int aabbMax1_1 = QuantizedBvhNodes.getCoord(aabbMax1, 1);
        int aabbMax1_2 = QuantizedBvhNodes.getCoord(aabbMax1, 2);

        int aabbMin2_0 = QuantizedBvhNodes.getCoord(aabbMin2, 0);
        int aabbMin2_1 = QuantizedBvhNodes.getCoord(aabbMin2, 1);
        int aabbMin2_2 = QuantizedBvhNodes.getCoord(aabbMin2, 2);

        int aabbMax2_0 = QuantizedBvhNodes.getCoord(aabbMax2, 0);
        int aabbMax2_1 = QuantizedBvhNodes.getCoord(aabbMax2, 1);
        int aabbMax2_2 = QuantizedBvhNodes.getCoord(aabbMax2, 2);

        boolean overlap;
        overlap = aabbMin1_0 <= aabbMax2_0 && aabbMax1_0 >= aabbMin2_0;
        overlap = aabbMin1_2 <= aabbMax2_2 && aabbMax1_2 >= aabbMin2_2 && overlap;
        overlap = aabbMin1_1 <= aabbMax2_1 && aabbMax1_1 >= aabbMin2_1 && overlap;
        return overlap;
    }

    protected void updateSubtreeHeaders(int leftChildNodexIndex, int rightChildNodexIndex) {
        assert (useQuantization);

        //btQuantizedBvhNode& leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
        int leftSubTreeSize = quantizedContiguousNodes.isLeafNode(leftChildNodexIndex) ? 1 : quantizedContiguousNodes.getEscapeIndex(leftChildNodexIndex);
        int leftSubTreeSizeInBytes = leftSubTreeSize * QuantizedBvhNodes.getNodeSize();

        //btQuantizedBvhNode& rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
        int rightSubTreeSize = quantizedContiguousNodes.isLeafNode(rightChildNodexIndex) ? 1 : quantizedContiguousNodes.getEscapeIndex(rightChildNodexIndex);
        int rightSubTreeSizeInBytes = rightSubTreeSize * QuantizedBvhNodes.getNodeSize();

        if (leftSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
            BvhSubtreeInfo subtree = new BvhSubtreeInfo();
            SubtreeHeaders.add(subtree);

            subtree.setAabbFromQuantizeNode(quantizedContiguousNodes, leftChildNodexIndex);
            subtree.rootNodeIndex = leftChildNodexIndex;
            subtree.subtreeSize = leftSubTreeSize;
        }

        if (rightSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
            BvhSubtreeInfo subtree = new BvhSubtreeInfo();
            SubtreeHeaders.add(subtree);

            subtree.setAabbFromQuantizeNode(quantizedContiguousNodes, rightChildNodexIndex);
            subtree.rootNodeIndex = rightChildNodexIndex;
            subtree.subtreeSize = rightSubTreeSize;
        }

        // PCK: update the copy of the size
        subtreeHeaderCount = SubtreeHeaders.size();
    }

    protected int sortAndCalcSplittingIndex(int startIndex, int endIndex, int splitAxis) {

        int splitIndex = startIndex;
        int numIndices = endIndex - startIndex;
        double splitValue;

        Vector3d means = Stack.newVec();
        means.set(0.0, 0.0, 0.0);
        Vector3d center = Stack.newVec();
        for (int i = startIndex; i < endIndex; i++) {
            center.add(getAabbMax(i), getAabbMin(i));
            center.scale(0.5);
            means.add(center);
        }
        means.scale(1.0 / (double) numIndices);

        splitValue = VectorUtil.getCoord(means, splitAxis);

        //sort leafNodes so all values larger than splitValue comes first, and smaller values start from 'splitIndex'.
        for (int i = startIndex; i < endIndex; i++) {
            center.add(getAabbMax(i), getAabbMin(i));
            center.scale(0.5);

            if (VectorUtil.getCoord(center, splitAxis) > splitValue) {
                // swap
                swapLeafNodes(i, splitIndex);
                splitIndex++;
            }
        }
        Stack.subVec(2);

        // if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
        // otherwise the tree-building might fail due to stack-overflows in certain cases.
        // unbalanced1 is unsafe: it can cause stack overflows
        // bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

        // unbalanced2 should work too: always use center (perfect balanced trees)
        // bool unbalanced2 = true;

        // this should be safe too:
        int rangeBalancedIndices = numIndices / 3;
        boolean unbalanced = ((splitIndex <= (startIndex + rangeBalancedIndices)) || (splitIndex >= (endIndex - 1 - rangeBalancedIndices)));

        if (unbalanced) {
            splitIndex = startIndex + (numIndices >> 1);
        }

        boolean unbalanced1 = (splitIndex == startIndex) || (splitIndex == (endIndex));
        assert (!unbalanced1);

        return splitIndex;
    }

    protected int calcSplittingAxis(int startIndex, int endIndex) {
        int i;

        Vector3d means = Stack.newVec();
        means.set(0.0, 0.0, 0.0);
        Vector3d variance = Stack.newVec();
        variance.set(0.0, 0.0, 0.0);
        int numIndices = endIndex - startIndex;

        Vector3d center = Stack.newVec();
        for (i = startIndex; i < endIndex; i++) {
            center.add(getAabbMax(i), getAabbMin(i));
            center.scale(0.5);
            means.add(center);
        }
        means.scale(1.0 / (double) numIndices);

        Vector3d diff2 = Stack.newVec();
        for (i = startIndex; i < endIndex; i++) {
            center.add(getAabbMax(i), getAabbMin(i));
            center.scale(0.5);
            diff2.sub(center, means);
            //diff2 = diff2 * diff2;
            VectorUtil.mul(diff2, diff2, diff2);
            variance.add(diff2);
        }
        variance.scale(1.0 / ((double) numIndices - 1));
        Stack.subVec(4);

        return VectorUtil.maxAxis(variance);
    }

    public void reportAabbOverlappingNodes(NodeOverlapCallback nodeCallback, Vector3d aabbMin, Vector3d aabbMax) {
        // either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

        if (useQuantization) {
            // quantize query AABB
            long quantizedQueryAabbMin;
            long quantizedQueryAabbMax;
            quantizedQueryAabbMin = quantizeWithClamp(aabbMin);
            quantizedQueryAabbMax = quantizeWithClamp(aabbMax);

            switch (traversalMode) {
                case STACKLESS:
                    walkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax, 0, curNodeIndex);
                    break;
                case RECURSIVE:
                    walkRecursiveQuantizedTreeAgainstQueryAabb(quantizedContiguousNodes, 0, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
                    break;
                default:
                    assert (false); // unsupported
            }
        } else {
            walkStacklessTree(nodeCallback, aabbMin, aabbMax);
        }
    }

    protected void walkStacklessTree(NodeOverlapCallback nodeCallback, Vector3d aabbMin, Vector3d aabbMax) {
        assert (!useQuantization);

        // JAVA NOTE: rewritten
        OptimizedBvhNode rootNode;//contiguousNodes.get(0);
        int rootNode_index = 0;

        int escapeIndex, curIndex = 0;
        int walkIterations = 0;
        boolean isLeafNode;
        //PCK: unsigned instead of bool
        //unsigned aabbOverlap;
        boolean aabbOverlap;

        while (curIndex < curNodeIndex) {
            // catch bugs in tree data
            assert (walkIterations < curNodeIndex);

            walkIterations++;

            rootNode = contiguousNodes.getQuick(rootNode_index);

            aabbOverlap = AabbUtil2.testAabbAgainstAabb2(aabbMin, aabbMax, rootNode.aabbMinOrg, rootNode.aabbMaxOrg);
            isLeafNode = (rootNode.escapeIndex == -1);

            // PCK: unsigned instead of bool
            if (isLeafNode && (aabbOverlap/* != 0*/)) {
                nodeCallback.processNode(rootNode.subPart, rootNode.triangleIndex);
            }

            //PCK: unsigned instead of bool
            if ((aabbOverlap/* != 0*/) || isLeafNode) {
                rootNode_index++;
                curIndex++;
            } else {
                escapeIndex = /*rootNode*/ contiguousNodes.getQuick(rootNode_index).escapeIndex;
                rootNode_index += escapeIndex;
                curIndex += escapeIndex;
            }
        }
        if (maxIterations < walkIterations) {
            maxIterations = walkIterations;
        }
    }

    protected void walkRecursiveQuantizedTreeAgainstQueryAabb(
            QuantizedBvhNodes currentNodes, int currentNodeId, NodeOverlapCallback nodeCallback,
            long quantizedQueryAabbMin, long quantizedQueryAabbMax) {
        assert (useQuantization);

        boolean isLeafNode;
        boolean aabbOverlap;

        aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                currentNodes.getQuantizedAabbMin(currentNodeId), currentNodes.getQuantizedAabbMax(currentNodeId));
        isLeafNode = currentNodes.isLeafNode(currentNodeId);

        if (aabbOverlap) {
            if (isLeafNode) {
                nodeCallback.processNode(currentNodes.getPartId(currentNodeId), currentNodes.getTriangleIndex(currentNodeId));
            } else {
                // process left and right children
                int leftChildNodeId = currentNodeId + 1;
                walkRecursiveQuantizedTreeAgainstQueryAabb(currentNodes, leftChildNodeId, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);

                int rightChildNodeId = currentNodes.isLeafNode(leftChildNodeId) ? leftChildNodeId + 1 : leftChildNodeId + currentNodes.getEscapeIndex(leftChildNodeId);
                walkRecursiveQuantizedTreeAgainstQueryAabb(currentNodes, rightChildNodeId, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
            }
        }
    }

    protected void walkStacklessQuantizedTreeAgainstRay(
            NodeOverlapCallback nodeCallback, Vector3d raySource, Vector3d rayTarget,
            Vector3d aabbMin, Vector3d aabbMax, @SuppressWarnings("SameParameterValue") int startNodeIndex, int endNodeIndex
    ) {
        assert (useQuantization);

        Vector3d tmp = Stack.newVec();

        int curIndex = startNodeIndex;
        int walkIterations = 0;
        int subTreeSize = endNodeIndex - startNodeIndex;

        QuantizedBvhNodes rootNode = quantizedContiguousNodes;
        int rootNode_idx = startNodeIndex;
        int escapeIndex;

        boolean isLeafNode;
        boolean boxBoxOverlap;
        boolean rayBoxOverlap;

        Vector3d rayDirection = Stack.newVec();
        tmp.sub(rayTarget, raySource);
        rayDirection.normalize(tmp);
        rayDirection.x = 1.0 / rayDirection.x;
        rayDirection.y = 1.0 / rayDirection.y;
        rayDirection.z = 1.0 / rayDirection.z;

        /* Quick pruning by quantized box */
        Vector3d rayAabbMin = Stack.newVec(raySource);
        Vector3d rayAabbMax = Stack.newVec(raySource);
        VectorUtil.setMin(rayAabbMin, rayTarget);
        VectorUtil.setMax(rayAabbMax, rayTarget);

        /* Add box cast extents to bounding box */
        rayAabbMin.add(aabbMin);
        rayAabbMax.add(aabbMax);

        long quantizedQueryAabbMin;
        long quantizedQueryAabbMax;
        quantizedQueryAabbMin = quantizeWithClamp(rayAabbMin);
        quantizedQueryAabbMax = quantizeWithClamp(rayAabbMax);

        Vector3d bounds_0 = Stack.newVec();
        Vector3d bounds_1 = Stack.newVec();
        Vector3d normal = Stack.newVec();
        double[] param = Stack.newDoublePtr();

        while (curIndex < endNodeIndex) {

            // catch bugs in tree data
            assert (walkIterations < subTreeSize);

            walkIterations++;
            // only interested if this is closer than any previous hit
            param[0] = 1.0;
            rayBoxOverlap = false;
            boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.getQuantizedAabbMin(rootNode_idx), rootNode.getQuantizedAabbMax(rootNode_idx));
            isLeafNode = rootNode.isLeafNode(rootNode_idx);
            if (boxBoxOverlap) {
                unQuantize(bounds_0, rootNode.getQuantizedAabbMin(rootNode_idx));
                unQuantize(bounds_1, rootNode.getQuantizedAabbMax(rootNode_idx));
                /* Add box cast extents */
                bounds_0.add(aabbMin);
                bounds_1.add(aabbMax);
                rayBoxOverlap = AabbUtil2.rayAabb(raySource, rayTarget, bounds_0, bounds_1, param, normal);
            }

            if (isLeafNode && rayBoxOverlap) {
                nodeCallback.processNode(rootNode.getPartId(rootNode_idx), rootNode.getTriangleIndex(rootNode_idx));
            }

            if (rayBoxOverlap || isLeafNode) {
                rootNode_idx++;
                curIndex++;
            } else {
                escapeIndex = rootNode.getEscapeIndex(rootNode_idx);
                rootNode_idx += escapeIndex;
                curIndex += escapeIndex;
            }
        }

        if (maxIterations < walkIterations) {
            maxIterations = walkIterations;
        }

        Stack.subVec(7);
        Stack.subDoublePtr(1);
    }

    protected void walkStacklessQuantizedTree(
            NodeOverlapCallback nodeCallback, long quantizedQueryAabbMin, long quantizedQueryAabbMax,
            @SuppressWarnings("SameParameterValue") int startNodeIndex, int endNodeIndex) {
        assert (useQuantization);

        int curIndex = startNodeIndex;
        int walkIterations = 0;
        int subTreeSize = endNodeIndex - startNodeIndex;

        QuantizedBvhNodes rootNode = quantizedContiguousNodes;
        int rootNode_idx = startNodeIndex;
        int escapeIndex;

        boolean isLeafNode;
        boolean aabbOverlap;

        while (curIndex < endNodeIndex) {

            // catch bugs in tree data
            assert (walkIterations < subTreeSize);

            walkIterations++;
            aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.getQuantizedAabbMin(rootNode_idx), rootNode.getQuantizedAabbMax(rootNode_idx));
            isLeafNode = rootNode.isLeafNode(rootNode_idx);

            if (isLeafNode && aabbOverlap) {
                nodeCallback.processNode(rootNode.getPartId(rootNode_idx), rootNode.getTriangleIndex(rootNode_idx));
            }

            if (aabbOverlap || isLeafNode) {
                rootNode_idx++;
                curIndex++;
            } else {
                escapeIndex = rootNode.getEscapeIndex(rootNode_idx);
                rootNode_idx += escapeIndex;
                curIndex += escapeIndex;
            }
        }

        if (maxIterations < walkIterations) {
            maxIterations = walkIterations;
        }
    }

    public void reportRayOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3d raySource, Vector3d rayTarget) {
        boolean fast_path = useQuantization && traversalMode == TraversalMode.STACKLESS;
        if (fast_path) {
            Vector3d tmp = Stack.newVec();
            tmp.set(0.0, 0.0, 0.0);
            walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, tmp, tmp, 0, curNodeIndex);
        } else {
            /* Otherwise fallback to AABB overlap test */
            Vector3d aabbMin = Stack.newVec(raySource);
            Vector3d aabbMax = Stack.newVec(raySource);
            VectorUtil.setMin(aabbMin, rayTarget);
            VectorUtil.setMax(aabbMax, rayTarget);
            reportAabbOverlappingNodes(nodeCallback, aabbMin, aabbMax);
        }
    }

    public void reportBoxCastOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3d raySource, Vector3d rayTarget, Vector3d aabbMin, Vector3d aabbMax) {
        boolean fast_path = useQuantization && traversalMode == TraversalMode.STACKLESS;
        if (fast_path) {
            walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, curNodeIndex);
        } else {
			/* Slow path:
			Construct the bounding box for the entire box cast and send that down the tree */
            Vector3d qaabbMin = Stack.newVec(raySource);
            Vector3d qaabbMax = Stack.newVec(raySource);
            VectorUtil.setMin(qaabbMin, rayTarget);
            VectorUtil.setMax(qaabbMax, rayTarget);
            qaabbMin.add(aabbMin);
            qaabbMax.add(aabbMax);
            reportAabbOverlappingNodes(nodeCallback, qaabbMin, qaabbMax);
            Stack.subVec(2);
        }
    }

    public long quantizeWithClamp(Vector3d point) {
        assert (useQuantization);

        double vx = (min(max(point.x, bvhAabbMin.x), bvhAabbMax.x) - bvhAabbMin.x) * bvhQuantization.x;
        double vy = (min(max(point.y, bvhAabbMin.y), bvhAabbMax.y) - bvhAabbMin.y) * bvhQuantization.y;
        double vz = (min(max(point.z, bvhAabbMin.z), bvhAabbMax.z) - bvhAabbMin.z) * bvhQuantization.z;

        int out0 = (int) (vx + 0.5) & 0xFFFF;
        int out1 = (int) (vy + 0.5) & 0xFFFF;
        int out2 = (int) (vz + 0.5) & 0xFFFF;

        return ((long) out0) | (((long) out1) << 16) | (((long) out2) << 32);
    }

    public void unQuantize(Vector3d vecOut, long vecIn) {
        int vecIn0 = (int) vecIn & 0xFFFF;
        int vecIn1 = (int) (vecIn >>> 16) & 0xFFFF;
        int vecIn2 = (int) (vecIn >>> 32) & 0xFFFF;

        vecOut.x = (double) vecIn0 / bvhQuantization.x;
        vecOut.y = (double) vecIn1 / bvhQuantization.y;
        vecOut.z = (double) vecIn2 / bvhQuantization.z;

        vecOut.add(bvhAabbMin);
    }

}
