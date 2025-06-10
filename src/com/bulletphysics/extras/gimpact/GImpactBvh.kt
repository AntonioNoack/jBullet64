package com.bulletphysics.extras.gimpact;

import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.extras.gimpact.BoxCollision.BoxBoxTransformCache;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
class GImpactBvh {

    private final BvhTree bvhTree = new BvhTree();
    private PrimitiveManagerBase primitiveManager;

    /**
     * This constructor doesn't build the tree. you must call buildSet.
     */
    public GImpactBvh() {
        primitiveManager = null;
    }

    /**
     * This constructor doesn't build the tree. you must call buildSet.
     */
    public GImpactBvh(PrimitiveManagerBase primitiveManager) {
        this.primitiveManager = primitiveManager;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AABB getGlobalBox(AABB out) {
        getNodeBound(0, out);
        return out;
    }

    public void setPrimitiveManager(PrimitiveManagerBase primitiveManager) {
        this.primitiveManager = primitiveManager;
    }

    public PrimitiveManagerBase getPrimitiveManager() {
        return primitiveManager;
    }

    // stackless refit
    protected void refit() {
        AABB leafbox = new AABB();
        AABB bound = new AABB();
        AABB temp_box = new AABB();

        int nodecount = getNodeCount();
        while ((nodecount--) != 0) {
            if (isLeafNode(nodecount)) {
                primitiveManager.getPrimitiveBox(getNodeData(nodecount), leafbox);
                setNodeBound(nodecount, leafbox);
            } else {
                //const BT_BVH_TREE_NODE * nodepointer = get_node_pointer(nodecount);
                //get left bound
                bound.invalidate();

                int child_node = getLeftNode(nodecount);
                if (child_node != 0) {
                    getNodeBound(child_node, temp_box);
                    bound.merge(temp_box);
                }

                child_node = getRightNode(nodecount);
                if (child_node != 0) {
                    getNodeBound(child_node, temp_box);
                    bound.merge(temp_box);
                }

                setNodeBound(nodecount, bound);
            }
        }
    }

    /**
     * This attemps to refit the box set.
     */
    public void update() {
        refit();
    }

    /**
     * This rebuild the entire set.
     */
    public void buildSet() {
        // obtain primitive boxes
        BvhDataArray primitiveBoxes = new BvhDataArray();
        primitiveBoxes.resize(primitiveManager.getPrimitiveCount());

        AABB tmpAABB = new AABB();

        for (int i = 0; i < primitiveBoxes.size(); i++) {
            //primitive_manager.get_primitive_box(i,primitive_boxes[i].bound);
            primitiveManager.getPrimitiveBox(i, tmpAABB);
            primitiveBoxes.setBounds(i, tmpAABB);

            primitiveBoxes.setData(i, i);
        }

        bvhTree.buildTree(primitiveBoxes);
    }

    /**
     * Returns the indices of the primitives in the primitive_manager field.
     */
    public boolean boxQuery(AABB box, IntArrayList collidedResults) {
        int curIndex = 0;
        int numNodes = getNodeCount();

        AABB bound = new AABB();

        while (curIndex < numNodes) {
            getNodeBound(curIndex, bound);

            // catch bugs in tree data

            boolean aabbOverlap = bound.hasCollision(box);
            boolean isLeafNode = isLeafNode(curIndex);

            if (isLeafNode && aabbOverlap) {
                collidedResults.add(getNodeData(curIndex));
            }

            if (aabbOverlap || isLeafNode) {
                // next subnode
                curIndex++;
            } else {
                // skip node
                curIndex += getEscapeNodeIndex(curIndex);
            }
        }
        return collidedResults.size() > 0;
    }

    /**
     * Returns the indices of the primitives in the primitive_manager field.
     */
    public boolean boxQueryTrans(AABB box, Transform transform, IntArrayList collided_results) {
        AABB transbox = new AABB(box);
        transbox.applyTransform(transform);
        return boxQuery(transbox, collided_results);
    }

    /**
     * Returns the indices of the primitives in the primitive_manager field.
     */
    public boolean rayQuery(Vector3d ray_dir, Vector3d ray_origin, IntArrayList collided_results) {
        int curIndex = 0;
        int numNodes = getNodeCount();

        AABB bound = new AABB();

        while (curIndex < numNodes) {
            getNodeBound(curIndex, bound);

            // catch bugs in tree data

            boolean aabbOverlap = bound.collideRay(ray_origin, ray_dir);
            boolean isleafnode = isLeafNode(curIndex);

            if (isleafnode && aabbOverlap) {
                collided_results.add(getNodeData(curIndex));
            }

            if (aabbOverlap || isleafnode) {
                // next subnode
                curIndex++;
            } else {
                // skip node
                curIndex += getEscapeNodeIndex(curIndex);
            }
        }
        return collided_results.size() > 0;
    }

    /**
     * Tells if this set has hierarchy.
     */
    public boolean hasHierarchy() {
        return true;
    }

    /**
     * Tells if this set is a trimesh.
     */
    public boolean isTrimesh() {
        return primitiveManager.isTrimesh();
    }

    public int getNodeCount() {
        return bvhTree.getNodeCount();
    }

    /**
     * Tells if the node is a leaf.
     */
    public boolean isLeafNode(int nodeIndex) {
        return bvhTree.isLeafNode(nodeIndex);
    }

    public int getNodeData(int nodeIndex) {
        return bvhTree.getNodeData(nodeIndex);
    }

    public void getNodeBound(int nodeIndex, AABB bound) {
        bvhTree.getNodeBound(nodeIndex, bound);
    }

    public void setNodeBound(int nodeIndex, AABB bound) {
        bvhTree.setNodeBound(nodeIndex, bound);
    }

    public int getLeftNode(int nodeIndex) {
        return bvhTree.getLeftNode(nodeIndex);
    }

    public int getRightNode(int nodeIndex) {
        return bvhTree.getRightNode(nodeIndex);
    }

    public int getEscapeNodeIndex(int nodeIndex) {
        return bvhTree.getEscapeNodeIndex(nodeIndex);
    }

    public void getNodeTriangle(int nodeIndex, PrimitiveTriangle triangle) {
        primitiveManager.getPrimitiveTriangle(getNodeData(nodeIndex), triangle);
    }

    public BvhTreeNodeArray getNodePointer() {
        return bvhTree.getNodePointer();
    }

    private static boolean nodeCollision(GImpactBvh boxset0, GImpactBvh boxset1, BoxBoxTransformCache trans_cache_1to0, int node0, int node1, boolean complete_primitive_tests) {
        AABB box0 = new AABB();
        boxset0.getNodeBound(node0, box0);
        AABB box1 = new AABB();
        boxset1.getNodeBound(node1, box1);

        return box0.overlappingTransCache(box1, trans_cache_1to0, complete_primitive_tests);
        //box1.appy_transform_trans_cache(trans_cache_1to0);
        //return box0.has_collision(box1);
    }

    /**
     * Stackless recursive collision routine.
     */
    private static void findCollisionPairsRecursive(GImpactBvh boxset0, GImpactBvh boxset1, IntPairList collision_pairs, BoxBoxTransformCache trans_cache_1to0, int node0, int node1, boolean complete_primitive_tests) {
        if (!nodeCollision(
                boxset0, boxset1, trans_cache_1to0,
                node0, node1, complete_primitive_tests)) {
            return;//avoid colliding internal nodes
        }
        if (boxset0.isLeafNode(node0)) {
            if (boxset1.isLeafNode(node1)) {
                // collision result
                collision_pairs.pushPair(boxset0.getNodeData(node0), boxset1.getNodeData(node1));
                return;
            } else {
                // collide left recursive
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        node0, boxset1.getLeftNode(node1), false);

                // collide right recursive
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        node0, boxset1.getRightNode(node1), false);
            }
        } else {
            if (boxset1.isLeafNode(node1)) {
                // collide left recursive
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        boxset0.getLeftNode(node0), node1, false);


                // collide right recursive
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        boxset0.getRightNode(node0), node1, false);
            } else {
                // collide left0 left1
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        boxset0.getLeftNode(node0), boxset1.getLeftNode(node1), false);

                // collide left0 right1
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        boxset0.getLeftNode(node0), boxset1.getRightNode(node1), false);

                // collide right0 left1
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        boxset0.getRightNode(node0), boxset1.getLeftNode(node1), false);

                // collide right0 right1
                findCollisionPairsRecursive(
                        boxset0, boxset1,
                        collision_pairs, trans_cache_1to0,
                        boxset0.getRightNode(node0), boxset1.getRightNode(node1), false);

            } // else if node1 is not a leaf
        } // else if node0 is not a leaf
    }

    public static void findCollision(GImpactBvh box0, Transform trans0, GImpactBvh box1, Transform trans1, IntPairList collisionPairs) {
        if (box0.getNodeCount() == 0 || box1.getNodeCount() == 0) return;

        BoxBoxTransformCache transCache1To0 = new BoxBoxTransformCache();
        transCache1To0.calcFromHomogenic(trans0, trans1);

        findCollisionPairsRecursive(
                box0, box1,
                collisionPairs, transCache1To0, 0, 0, true);
    }

}
