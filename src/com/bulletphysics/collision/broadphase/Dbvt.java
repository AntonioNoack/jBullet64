package com.bulletphysics.collision.broadphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;

import java.util.ArrayList;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import java.util.Collections;

/**
 * @author jezek2
 * Dbvt implementation by Nathanael Presson
 */
public class Dbvt {

    public static final int SIMPLE_STACK_SIZE = 64;
    public static final int DOUBLE_STACK_SIZE = SIMPLE_STACK_SIZE * 2;

    public Node root = null;
    public Node free = null;
    public int lkhd = -1;
    public int leaves = 0;
    public /*unsigned*/ int oPath = 0;

    public Dbvt() {
    }

    public void clear() {
        if (root != null) {
            recurseDeleteNode(this, root);
        }
        //btAlignedFree(m_free);
        free = null;
    }

    public boolean empty() {
        return (root == null);
    }

    public void optimizeBottomUp() {
        if (root != null) {
            ArrayList<Node> leaves = new ArrayList<>(this.leaves);
            fetchleaves(this, root, leaves);
            bottomup(this, leaves);
            root = leaves.get(0);
        }
    }

    public void optimizeTopDown() {
        optimizeTopDown(128);
    }

    public void optimizeTopDown(int bu_treshold) {
        if (root != null) {
            ArrayList<Node> leaves = new ArrayList<>(this.leaves);
            fetchleaves(this, root, leaves);
            root = topdown(this, leaves, bu_treshold);
        }
    }

    public void optimizeIncremental(int passes) {
        if (passes < 0) {
            passes = leaves;
        }

        if (root != null && (passes > 0)) {
            Node[] root_ref = new Node[1];
            do {
                Node node = root;
                int bit = 0;
                while (node.isInternal()) {
                    root_ref[0] = root;
                    node = sort(node, root_ref).childs[(oPath >>> bit) & 1];
                    root = root_ref[0];

                    bit = (bit + 1) & (/*sizeof(unsigned)*/4 * 8 - 1);
                }
                update(node);
                ++oPath;
            }
            while ((--passes) != 0);
        }
    }

    public Node insert(DbvtAabbMm box, Object data) {
        Node leaf = createNode(this, null, box, data);
        insertLeaf(this, root, leaf);
        leaves++;
        return leaf;
    }

    public void update(Node leaf) {
        update(leaf, -1);
    }

    public void update(Node leaf, int lookahead) {
        Node root = removeLeaf(this, leaf);
        if (root != null) {
            if (lookahead >= 0) {
                for (int i = 0; (i < lookahead) && root.parent != null; i++) {
                    root = root.parent;
                }
            } else {
                root = this.root;
            }
        }
        insertLeaf(this, root, leaf);
    }

    public void update(Node leaf, DbvtAabbMm volume) {
        Node root = removeLeaf(this, leaf);
        if (root != null) {
            if (lkhd >= 0) {
                for (int i = 0; (i < lkhd) && root.parent != null; i++) {
                    root = root.parent;
                }
            } else {
                root = this.root;
            }
        }
        leaf.volume.set(volume);
        insertLeaf(this, root, leaf);
    }

    public boolean update(Node leaf, DbvtAabbMm volume, Vector3d velocity, double margin) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        Vector3d tmp = Stack.newVec();
        tmp.set(margin, margin, margin);
        volume.Expand(tmp);
        volume.SignedExpand(velocity);
        update(leaf, volume);
        Stack.subVec(1);
        return true;
    }

    public boolean update(Node leaf, DbvtAabbMm volume, Vector3d velocity) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        volume.SignedExpand(velocity);
        update(leaf, volume);
        return true;
    }

    public boolean update(Node leaf, DbvtAabbMm volume, double margin) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        Vector3d tmp = Stack.newVec();
        tmp.set(margin, margin, margin);
        volume.Expand(tmp);
        update(leaf, volume);
        Stack.subVec(1);
        return true;
    }

    public void remove(Node leaf) {
        removeLeaf(this, leaf);
        deleteNode(this, leaf);
        leaves--;
    }

    /*public void write(IWriter iwriter) {
        throw new UnsupportedOperationException();
    }*/

    /*public void clone(Dbvt dest) {
        clone(dest, null);
    }

    public void clone(Dbvt dest, IClone iclone) {
        throw new UnsupportedOperationException();
    }*/

    public static int countLeaves(Node node) {
        if (node.isInternal()) {
            return countLeaves(node.childs[0]) + countLeaves(node.childs[1]);
        } else {
            return 1;
        }
    }

    public static void extractLeaves(Node node, ArrayList<Node> leaves) {
        if (node.isInternal()) {
            extractLeaves(node.childs[0], leaves);
            extractLeaves(node.childs[1], leaves);
        } else {
            leaves.add(node);
        }
    }

    public static void enumNodes(Node root, ICollide policy) {
        //DBVT_CHECKTYPE
        policy.Process(root);
        if (root.isInternal()) {
            enumNodes(root.childs[0], policy);
            enumNodes(root.childs[1], policy);
        }
    }

    public static void enumLeaves(Node root, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root.isInternal()) {
            enumLeaves(root.childs[0], policy);
            enumLeaves(root.childs[1], policy);
        } else {
            policy.Process(root);
        }
    }

    public static void collideTT(Node root0, Node root1, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 != null && root1 != null) {
            ArrayList<Node> stack = new ArrayList<>(DOUBLE_STACK_SIZE);
            stack.add(root0);
            stack.add(root1);
            do {
                Node pb = stack.remove(stack.size() - 1);
                Node pa = stack.remove(stack.size() - 1);
                if (pa == pb) {
                    if (pa.isInternal()) {
                        stack.add(pa.childs[0]);
                        stack.add(pa.childs[0]);
                        stack.add(pa.childs[1]);
                        stack.add(pa.childs[1]);
                        stack.add(pa.childs[0]);
                        stack.add(pa.childs[1]);
                    }
                } else if (DbvtAabbMm.Intersect(pa.volume, pb.volume)) {
                    if (pa.isInternal()) {
                        if (pb.isInternal()) {
                            stack.add(pa.childs[0]);
                            stack.add(pb.childs[0]);
                            stack.add(pa.childs[1]);
                            stack.add(pb.childs[0]);
                            stack.add(pa.childs[0]);
                            stack.add(pb.childs[1]);
                            stack.add(pa.childs[1]);
                            stack.add(pb.childs[1]);
                        } else {
                            stack.add(pa.childs[0]);
                            stack.add(pb);
                            stack.add(pa.childs[1]);
                            stack.add(pb);
                        }
                    } else {
                        if (pb.isInternal()) {
                            stack.add(pa);
                            stack.add(pb.childs[0]);
                            stack.add(pa);
                            stack.add(pb.childs[1]);
                        } else {
                            policy.Process(pa, pb);
                        }
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static void collideTT(Node root0, Node root1, Transform xform, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 != null && root1 != null) {
            ArrayList<Node> stack = new ArrayList<>(DOUBLE_STACK_SIZE);
            stack.add(root0);stack.add(root1);
            do {
                Node pb = stack.remove(stack.size() - 1);
                Node pa = stack.remove(stack.size() - 1);
                if (pa == pb) {
                    if (pa.isInternal()) {
                        stack.add(pa.childs[0]);
                        stack.add(pa.childs[0]);
                        stack.add(pa.childs[1]);
                        stack.add(pa.childs[1]);
                        stack.add(pa.childs[0]);
                        stack.add(pa.childs[1]);
                    }
                } else if (DbvtAabbMm.Intersect(pa.volume, pb.volume, xform)) {
                    if (pa.isInternal()) {
                        if (pb.isInternal()) {
                            stack.add(pa.childs[0]);
                            stack.add(pb.childs[0]);
                            stack.add(pa.childs[1]);
                            stack.add(pb.childs[0]);
                            stack.add(pa.childs[0]);
                            stack.add(pb.childs[1]);
                            stack.add(pa.childs[1]);
                            stack.add(pb.childs[1]);
                        } else {
                            stack.add(pa.childs[0]);
                            stack.add(pb);
                            stack.add(pa.childs[1]);
                            stack.add(pb);
                        }
                    } else {
                        if (pb.isInternal()) {
                            stack.add(pa);
                            stack.add(pb.childs[0]);
                            stack.add(pa);
                            stack.add(pb.childs[1]);
                        } else {
                            policy.Process(pa, pb);
                        }
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static void collideTT(Node root0, Transform xform0, Node root1, Transform xform1, ICollide policy) {
        Transform xform = Stack.borrowTrans();
        xform.inverse(xform0);
        xform.mul(xform1);
        collideTT(root0, root1, xform, policy);
    }

    public static void collideTV(Node root, DbvtAabbMm volume, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            ArrayList<Node> stack = new ArrayList<>(SIMPLE_STACK_SIZE);
            stack.add(root);
            do {
                Node n = stack.remove(stack.size() - 1);
                if (DbvtAabbMm.Intersect(n.volume, volume)) {
                    if (n.isInternal()) {
                        stack.add(n.childs[0]);
                        stack.add(n.childs[1]);
                    } else {
                        policy.Process(n);
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static void collideRAY(Node root, Vector3d origin, Vector3d direction, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            Vector3d normal = Stack.newVec();
            normal.normalize(direction);
            Vector3d invDir = Stack.newVec();
            invDir.set(1.0 / normal.x, 1.0 / normal.y, 1.0 / normal.z);
            int[] signs = new int[]{direction.x < 0 ? 1 : 0, direction.y < 0 ? 1 : 0, direction.z < 0 ? 1 : 0};
            ArrayList<Node> stack = new ArrayList<>(SIMPLE_STACK_SIZE);
            stack.add(root);
            do {
                Node node = stack.remove(stack.size() - 1);
                if (DbvtAabbMm.Intersect(node.volume, origin, invDir, signs)) {
                    if (node.isInternal()) {
                        stack.add(node.childs[0]);
                        stack.add(node.childs[1]);
                    } else {
                        policy.Process(node);
                    }
                }
            }
            while (stack.size() != 0);
            Stack.subVec(2);
        }
    }

    public static void collideKDOP(Node root, Vector3d[] normals, double[] offsets, int count, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            int inside = (1 << count) - 1;
            ArrayList<sStkNP> stack = new ArrayList<>(SIMPLE_STACK_SIZE);
            int[] signs = new int[4 * 8];
            assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
            for (int i = 0; i < count; ++i) {
                signs[i] = ((normals[i].x >= 0) ? 1 : 0) +
                        ((normals[i].y >= 0) ? 2 : 0) +
                        ((normals[i].z >= 0) ? 4 : 0);
            }
            stack.add(new sStkNP(root, 0));
            do {
                sStkNP se = stack.remove(stack.size() - 1);
                boolean out = false;
                for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
                    if (0 == (se.mask & j)) {
                        int side = se.node.volume.Classify(normals[i], offsets[i], signs[i]);
                        switch (side) {
                            case -1:
                                out = true;
                                break;
                            case +1:
                                se.mask |= j;
                                break;
                        }
                    }
                }
                if (!out) {
                    if ((se.mask != inside) && (se.node.isInternal())) {
                        stack.add(new sStkNP(se.node.childs[0], se.mask));
                        stack.add(new sStkNP(se.node.childs[1], se.mask));
                    } else {
                        if (policy.AllLeaves(se.node)) {
                            enumLeaves(se.node, policy);
                        }
                    }
                }
            }
            while (stack.size() != 0);
        }
    }

    public static void collideOCL(Node root, Vector3d[] normals, double[] offsets, Vector3d sortaxis, int count, ICollide policy) {
        collideOCL(root, normals, offsets, sortaxis, count, policy, true);
    }

    public static void collideOCL(Node root, Vector3d[] normals, double[] offsets, Vector3d sortaxis, int count, ICollide policy, boolean fullsort) {
        //DBVT_CHECKTYPE
        if (root != null) {
            int srtsgns = (sortaxis.x >= 0 ? 1 : 0) +
                    (sortaxis.y >= 0 ? 2 : 0) +
                    (sortaxis.z >= 0 ? 4 : 0);
            int inside = (1 << count) - 1;
            ArrayList<sStkNPS> stock = new ArrayList<>();
            IntArrayList ifree = new IntArrayList();
            IntArrayList stack = new IntArrayList();
            int[] signs = new int[/*sizeof(unsigned)*8*/4 * 8];
            assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
            for (int i = 0; i < count; i++) {
                signs[i] = ((normals[i].x >= 0) ? 1 : 0) +
                        ((normals[i].y >= 0) ? 2 : 0) +
                        ((normals[i].z >= 0) ? 4 : 0);
            }
            //stock.reserve(SIMPLE_STACKSIZE);
            //stack.reserve(SIMPLE_STACKSIZE);
            //ifree.reserve(SIMPLE_STACKSIZE);
            stack.add(allocate(ifree, stock, new sStkNPS(root, 0, root.volume.ProjectMinimum(sortaxis, srtsgns))));
            do {
                // JAVA NOTE: check
                int id = stack.remove(stack.size() - 1);
                sStkNPS se = stock.get(id);
                ifree.add(id);
                if (se.mask != inside) {
                    boolean out = false;
                    for (int i = 0, j = 1; (!out) && (i < count); ++i, j <<= 1) {
                        if (0 == (se.mask & j)) {
                            int side = se.node.volume.Classify(normals[i], offsets[i], signs[i]);
                            switch (side) {
                                case -1:
                                    out = true;
                                    break;
                                case +1:
                                    se.mask |= j;
                                    break;
                            }
                        }
                    }
                    if (out) {
                        continue;
                    }
                }
                if (policy.Descent(se.node)) {
                    if (se.node.isInternal()) {
                        Node[] pns = new Node[]{se.node.childs[0], se.node.childs[1]};
                        sStkNPS[] nes = new sStkNPS[]{new sStkNPS(pns[0], se.mask, pns[0].volume.ProjectMinimum(sortaxis, srtsgns)),
                                new sStkNPS(pns[1], se.mask, pns[1].volume.ProjectMinimum(sortaxis, srtsgns))
                        };
                        int q = nes[0].value < nes[1].value ? 1 : 0;
                        int j = stack.size();
                        if (fullsort && (j > 0)) {
                            /* Insert 0	*/
                            j = nearest(stack, stock, nes[q].value, 0, stack.size());
                            stack.add(0);
                            //#if DBVT_USE_MEMMOVE
                            //memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
                            //#else
                            for (int k = stack.size() - 1; k > j; --k) {
                                stack.set(k, stack.get(k - 1));
                                //#endif
                            }
                            stack.set(j, allocate(ifree, stock, nes[q]));
                            /* Insert 1	*/
                            j = nearest(stack, stock, nes[1 - q].value, j, stack.size());
                            stack.add(0);
                            //#if DBVT_USE_MEMMOVE
                            //memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
                            //#else
                            for (int k = stack.size() - 1; k > j; --k) {
                                stack.set(k, stack.get(k - 1));
                                //#endif
                            }
                            stack.set(j, allocate(ifree, stock, nes[1 - q]));
                        } else {
                            stack.add(allocate(ifree, stock, nes[q]));
                            stack.add(allocate(ifree, stock, nes[1 - q]));
                        }
                    } else {
                        policy.Process(se.node, se.value);
                    }
                }
            }
            while (stack.size() != 0);
        }
    }

    public static void collideTU(Node root, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            ArrayList<Node> stack = new ArrayList<>(SIMPLE_STACK_SIZE);
            stack.add(root);
            do {
                Node n = stack.remove(stack.size() - 1);
                if (policy.Descent(n)) {
                    if (n.isInternal()) {
                        stack.add(n.childs[0]);
                        stack.add(n.childs[1]);
                    } else {
                        policy.Process(n);
                    }
                }
            }
            while (stack.size() > 0);
        }
    }

    public static int nearest(IntArrayList i, ArrayList<sStkNPS> a, double v, int l, int h) {
        int m = 0;
        while (l < h) {
            m = (l + h) >> 1;
            if (a.get(i.get(m)).value >= v) {
                l = m + 1;
            } else {
                h = m;
            }
        }
        return h;
    }

    public static int allocate(IntArrayList ifree, ArrayList<sStkNPS> stock, sStkNPS value) {
        int i;
        if (ifree.size() > 0) {
            i = ifree.get(ifree.size() - 1);
            ifree.remove(ifree.size() - 1);
            stock.get(i).set(value);
        } else {
            i = stock.size();
            stock.add(value);
        }
        return (i);
    }

    ////////////////////////////////////////////////////////////////////////////

    private static int indexOf(Node node) {
        return (node.parent.childs[1] == node) ? 1 : 0;
    }

    private static DbvtAabbMm merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm out) {
        DbvtAabbMm.Merge(a, b, out);
        return out;
    }

    // volume+edge lengths
    private static double size(DbvtAabbMm a) {
        Vector3d edges = a.Lengths(Stack.borrowVec());
        return (edges.x * edges.y * edges.z +
                edges.x + edges.y + edges.z);
    }

    private static void deleteNode(Dbvt pdbvt, Node node) {
        //btAlignedFree(pdbvt->m_free);
        pdbvt.free = node;
    }

    private static void recurseDeleteNode(Dbvt pdbvt, Node node) {
        if (node.isInternal()) {
            recurseDeleteNode(pdbvt, node.childs[0]);
            recurseDeleteNode(pdbvt, node.childs[1]);
        }
        if (node == pdbvt.root) {
            pdbvt.root = null;
        }
        deleteNode(pdbvt, node);
    }

    private static Node createNode(Dbvt pdbvt, Node parent, DbvtAabbMm volume, Object data) {
        Node node;
        if (pdbvt.free != null) {
            node = pdbvt.free;
            pdbvt.free = null;
        } else {
            node = new Node();
        }
        node.parent = parent;
        node.volume.set(volume);
        node.data = data;
        node.childs[1] = null;
        return node;
    }

    private static void insertLeaf(Dbvt pdbvt, Node root, Node leaf) {
        if (pdbvt.root == null) {
            pdbvt.root = leaf;
            leaf.parent = null;
        } else {
            if (root.isInternal()) {
                do {
                    if (DbvtAabbMm.Proximity(root.childs[0].volume, leaf.volume) <
                            DbvtAabbMm.Proximity(root.childs[1].volume, leaf.volume)) {
                        root = root.childs[0];
                    } else {
                        root = root.childs[1];
                    }
                } while (root.isInternal());
            }
            Node prev = root.parent;
            Node node = createNode(pdbvt, prev, merge(leaf.volume, root.volume, Stack.borrowAABBMM), null);
            if (prev != null) {
                prev.childs[indexOf(root)] = node;
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                do {
                    if (!prev.volume.Contain(node.volume)) {
                        DbvtAabbMm.Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
                    } else {
                        break;
                    }
                    node = prev;
                } while (null != (prev = node.parent));
            } else {
                node.childs[0] = root;
                root.parent = node;
                node.childs[1] = leaf;
                leaf.parent = node;
                pdbvt.root = node;
            }
        }
    }

    private static Node removeLeaf(Dbvt pdbvt, Node leaf) {
        if (leaf == pdbvt.root) {
            pdbvt.root = null;
            return null;
        } else {
            Node parent = leaf.parent;
            Node prev = parent.parent;
            Node sibling = parent.childs[1 - indexOf(leaf)];
            if (prev != null) {
                prev.childs[indexOf(parent)] = sibling;
                sibling.parent = prev;
                deleteNode(pdbvt, parent);
                while (prev != null) {
                    DbvtAabbMm pb = prev.volume;
                    DbvtAabbMm.Merge(prev.childs[0].volume, prev.childs[1].volume, prev.volume);
                    if (DbvtAabbMm.NotEqual(pb, prev.volume)) {
                        prev = prev.parent;
                    } else {
                        break;
                    }
                }
                return (prev != null ? prev : pdbvt.root);
            } else {
                pdbvt.root = sibling;
                sibling.parent = null;
                deleteNode(pdbvt, parent);
                return pdbvt.root;
            }
        }
    }

    private static void fetchleaves(Dbvt pdbvt, Node root, ArrayList<Node> leaves) {
        fetchleaves(pdbvt, root, leaves, -1);
    }

    private static void fetchleaves(Dbvt pdbvt, Node root, ArrayList<Node> leaves, int depth) {
        if (root.isInternal() && depth != 0) {
            fetchleaves(pdbvt, root.childs[0], leaves, depth - 1);
            fetchleaves(pdbvt, root.childs[1], leaves, depth - 1);
            deleteNode(pdbvt, root);
        } else {
            leaves.add(root);
        }
    }

    private static void split(ArrayList<Node> leaves, ArrayList<Node> left, ArrayList<Node> right, Vector3d org, Vector3d axis) {
        Vector3d tmp = Stack.newVec();
        MiscUtil.resize(left, 0, Node.class);
        MiscUtil.resize(right, 0, Node.class);
        for (int i = 0, ni = leaves.size(); i < ni; i++) {
            leaves.get(i).volume.Center(tmp);
            tmp.sub(org);
            if (axis.dot(tmp) < 0.0) {
                left.add(leaves.get(i));
            } else {
                right.add(leaves.get(i));
            }
        }
        Stack.subVec(1);
    }

    private static DbvtAabbMm bounds(ArrayList<Node> leaves) {
        DbvtAabbMm volume = new DbvtAabbMm(leaves.get(0).volume);
        for (int i = 1, ni = leaves.size(); i < ni; i++) {
            merge(volume, leaves.get(i).volume, volume);
        }
        return volume;
    }

    private static void bottomup(Dbvt pdbvt, ArrayList<Node> leaves) {
        DbvtAabbMm tmpVolume = new DbvtAabbMm();
        while (leaves.size() > 1) {
            double minsize = BulletGlobals.SIMD_INFINITY;
            int[] minidx = new int[]{-1, -1};
            for (int i = 0; i < leaves.size(); i++) {
                for (int j = i + 1; j < leaves.size(); j++) {
                    double sz = size(merge(leaves.get(i).volume, leaves.get(j).volume, tmpVolume));
                    if (sz < minsize) {
                        minsize = sz;
                        minidx[0] = i;
                        minidx[1] = j;
                    }
                }
            }
            Node[] n = new Node[]{leaves.get(minidx[0]), leaves.get(minidx[1])};
            Node p = createNode(pdbvt, null, merge(n[0].volume, n[1].volume, Stack.borrowAABBMM), null);
            p.childs[0] = n[0];
            p.childs[1] = n[1];
            n[0].parent = p;
            n[1].parent = p;
            // JAVA NOTE: check
            leaves.set(minidx[0], p);
            Collections.swap(leaves, minidx[1], leaves.size() - 1);
            leaves.remove(leaves.size() - 1);
        }
    }

    private static final Vector3d[] axis = new Vector3d[]{new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, 0, 1)};

    private static Node topdown(Dbvt pdbvt, ArrayList<Node> leaves, int bu_treshold) {
        if (leaves.size() > 1) {
            if (leaves.size() > bu_treshold) {
                DbvtAabbMm vol = bounds(leaves);
                Vector3d org = vol.Center(Stack.newVec());
                ArrayList<Node>[] sets = new ArrayList[2];
                for (int i = 0; i < sets.length; i++) {
                    sets[i] = new ArrayList();
                }
                int bestaxis = -1;
                int bestmidp = leaves.size();
                int[] splitCount = new int[6];

                Vector3d x = Stack.newVec();

                for (int i = 0; i < leaves.size(); i++) {
                    leaves.get(i).volume.Center(x);
                    x.sub(org);
                    for (int j = 0; j < 3; j++) {
                        splitCount[j * 2 + (x.dot(axis[j]) > 0.0 ? 1 : 0)]++;
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if ((splitCount[i * 2] > 0) && (splitCount[i * 2 + 1] > 0)) {
                        int midp = Math.abs(splitCount[i * 2] - splitCount[i * 2 + 1]);
                        if (midp < bestmidp) {
                            bestaxis = i;
                            bestmidp = midp;
                        }
                    }
                }
                if (bestaxis >= 0) {
                    split(leaves, sets[0], sets[1], org, axis[bestaxis]);
                } else {
                    for (int i = 0, ni = leaves.size(); i < ni; i++) {
                        sets[i & 1].add(leaves.get(i));
                    }
                }
                Node node = createNode(pdbvt, null, vol, null);
                node.childs[0] = topdown(pdbvt, sets[0], bu_treshold);
                node.childs[1] = topdown(pdbvt, sets[1], bu_treshold);
                node.childs[0].parent = node;
                node.childs[1].parent = node;
                return node;
            } else {
                bottomup(pdbvt, leaves);
                return leaves.get(0);
            }
        }
        return leaves.get(0);
    }

    private static Node sort(Node n, Node[] r) {
        Node p = n.parent;
        assert (n.isInternal());
        // JAVA TODO: fix this
        if (p != null && p.hashCode() > n.hashCode()) {
            int i = indexOf(n);
            int j = 1 - i;
            Node s = p.childs[j];
            Node q = p.parent;
            assert (n == p.childs[i]);
            if (q != null) {
                q.childs[indexOf(p)] = n;
            } else {
                r[0] = n;
            }
            s.parent = n;
            p.parent = n;
            n.parent = q;
            p.childs[0] = n.childs[0];
            p.childs[1] = n.childs[1];
            n.childs[0].parent = p;
            n.childs[1].parent = p;
            n.childs[i] = p;
            n.childs[j] = s;

            DbvtAabbMm.swap(p.volume, n.volume);
            return p;
        }
        return n;
    }

    public static class Node {
        public final DbvtAabbMm volume = new DbvtAabbMm();
        public Node parent;
        public final Node[] childs = new Node[2];
        public Object data;

        public boolean isLeaf() {
            return childs[1] == null;
        }

        public boolean isInternal() {
            return childs[1] != null;
        }
    }

    public static class sStkNP {
        public Node node;
        public int mask;

        public sStkNP(Node n, int m) {
            node = n;
            mask = m;
        }
    }

    public static class sStkNPS {
        public Node node;
        public int mask;
        public double value;

        public sStkNPS(Node n, int m, double v) {
            node = n;
            mask = m;
            value = v;
        }

        public void set(sStkNPS o) {
            node = o.node;
            mask = o.mask;
            value = o.value;
        }
    }

    public static class ICollide {
        public void Process(Node n1, Node n2) {
        }

        public void Process(Node n) {
        }

        public void Process(Node n, double f) {
            Process(n);
        }

        public boolean Descent(Node n) {
            return true;
        }

        public boolean AllLeaves(Node n) {
            return true;
        }
    }

}
