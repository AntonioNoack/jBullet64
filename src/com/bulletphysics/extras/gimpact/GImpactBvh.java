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

    protected BvhTree box_tree = new BvhTree();
    protected PrimitiveManagerBase primitive_manager;

    /**
     * This constructor doesn't build the tree. you must call buildSet.
     */
    public GImpactBvh() {
        primitive_manager = null;
    }

    /**
     * This constructor doesn't build the tree. you must call buildSet.
     */
    public GImpactBvh(PrimitiveManagerBase primitive_manager) {
        this.primitive_manager = primitive_manager;
    }

    public AABB getGlobalBox(AABB out) {
        getNodeBound(0, out);
        return out;
    }

    public void setPrimitiveManager(PrimitiveManagerBase primitive_manager) {
        this.primitive_manager = primitive_manager;
    }

    public PrimitiveManagerBase getPrimitiveManager() {
        return primitive_manager;
    }

    // stackless refit
    protected void refit() {
        AABB leafbox = new AABB();
        AABB bound = new AABB();
        AABB temp_box = new AABB();

        int nodecount = getNodeCount();
        while ((nodecount--) != 0) {
            if (isLeafNode(nodecount)) {
                primitive_manager.getPrimitiveBox(getNodeData(nodecount), leafbox);
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
        BvhDataArray primitive_boxes = new BvhDataArray();
        primitive_boxes.resize(primitive_manager.getPrimitiveCount());

        AABB tmpAABB = new AABB();

        for (int i = 0; i < primitive_boxes.size(); i++) {
            //primitive_manager.get_primitive_box(i,primitive_boxes[i].bound);
            primitive_manager.getPrimitiveBox(i, tmpAABB);
            primitive_boxes.setBound(i, tmpAABB);

            primitive_boxes.setData(i, i);
        }

        box_tree.buildTree(primitive_boxes);
    }

    /**
     * Returns the indices of the primitives in the primitive_manager field.
     */
    public boolean boxQuery(AABB box, IntArrayList collided_results) {
        int curIndex = 0;
        int numNodes = getNodeCount();

        AABB bound = new AABB();

        while (curIndex < numNodes) {
            getNodeBound(curIndex, bound);

            // catch bugs in tree data

            boolean aabbOverlap = bound.hasCollision(box);
            boolean lsLeafNode = isLeafNode(curIndex);

            if (lsLeafNode && aabbOverlap) {
                collided_results.add(getNodeData(curIndex));
            }

            if (aabbOverlap || lsLeafNode) {
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
     * Returns the indices of the primitives in the primitive_manager field.
     */
    public boolean boxQueryTrans(AABB box, Transform transform, IntArrayList collided_results) {
        AABB transBox = new AABB(box);
        transBox.applyTransform(transform);
        return boxQuery(transBox, collided_results);
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

            boolean aabbOverlap = bound.collide_ray(ray_origin, ray_dir);
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
        if (collided_results.size() > 0) {
            return true;
        }
        return false;
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
        return primitive_manager.isTrimesh();
    }

    public int getNodeCount() {
        return box_tree.getNodeCount();
    }

    /**
     * Tells if the node is a leaf.
     */
    public boolean isLeafNode(int nodeIndex) {
        return box_tree.isLeafNode(nodeIndex);
    }

    public int getNodeData(int nodeIndex) {
        return box_tree.getNodeData(nodeIndex);
    }

    public void getNodeBound(int nodeIndex, AABB bound) {
        box_tree.getNodeBound(nodeIndex, bound);
    }

    public void setNodeBound(int nodeIndex, AABB bound) {
        box_tree.setNodeBound(nodeIndex, bound);
    }

    public int getLeftNode(int nodeIndex) {
        return box_tree.getLeftNode(nodeIndex);
    }

    public int getRightNode(int nodeIndex) {
        return box_tree.getRightNode(nodeIndex);
    }

    public int getEscapeNodeIndex(int nodeIndex) {
        return box_tree.getEscapeNodeIndex(nodeIndex);
    }

    public void getNodeTriangle(int nodeIndex, PrimitiveTriangle triangle) {
        primitive_manager.getPrimitiveTriangle(getNodeData(nodeIndex), triangle);
    }

    public BvhTreeNodeArray getNodePointer() {
        return box_tree.get_node_pointer();
    }

    private static boolean nodeCollision(GImpactBvh boxSet0, GImpactBvh boxSet1, BoxBoxTransformCache trans_cache_1to0, int node0, int node1, boolean complete_primitive_tests) {
        AABB box0 = new AABB();
        boxSet0.getNodeBound(node0, box0);
        AABB box1 = new AABB();
        boxSet1.getNodeBound(node1, box1);
        return box0.overlappingTransCache(box1, trans_cache_1to0, complete_primitive_tests);
    }

    /**
     * Stackless recursive collision routine.
     */
    private static void findCollisionPairsRecursive(GImpactBvh boxset0, GImpactBvh boxset1, IntPairArrayList collision_pairs, BoxBoxTransformCache trans_cache_1to0, int node0, int node1, boolean complete_primitive_tests) {
        if (!nodeCollision(
                boxset0, boxset1, trans_cache_1to0,
                node0, node1, complete_primitive_tests)) {
            return;//avoid colliding internal nodes
        }
        if (boxset0.isLeafNode(node0)) {
            if (boxset1.isLeafNode(node1)) {
                // collision result
                collision_pairs.pushPair(boxset0.getNodeData(node0), boxset1.getNodeData(node1));
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

    //public static double getAverageTreeCollisionTime();

    public static void findCollision(GImpactBvh boxSet0, Transform trans0, GImpactBvh boxSet1, Transform trans1, IntPairArrayList collision_pairs) {
        if (boxSet0.getNodeCount() > 0 && boxSet1.getNodeCount() > 0) {
            BoxBoxTransformCache transCache1To0 = new BoxBoxTransformCache();

            transCache1To0.calcFromHomogenic(trans0, trans1);

            findCollisionPairsRecursive(
                    boxSet0, boxSet1,
                    collision_pairs, transCache1To0, 0, 0, true);

        }
    }

}
