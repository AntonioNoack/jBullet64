// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import java.util.Collections;

/**
 * @author jezek2
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

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean isEmpty() {
        return (root == null);
    }

    public void optimizeBottomUp() {
        if (root != null) {
            ObjectArrayList<Node> leaves = new ObjectArrayList<Node>(this.leaves);
            fetchLeaves(this, root, leaves);
            bottomUp(this, leaves);
            root = leaves.getQuick(0);
        }
    }

    public void optimizeTopDown() {
        optimizeTopDown(128);
    }

    public void optimizeTopDown(int bu_treshold) {
        if (root != null) {
            ObjectArrayList<Node> leaves = new ObjectArrayList<Node>(this.leaves);
            fetchLeaves(this, root, leaves);
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
                    node = sort(node, root_ref);
                    node = ((oPath >>> bit) & 1) == 0 ? node.child0 : node.child1;
                    root = root_ref[0];

                    bit = (bit + 1) & (/*sizeof(unsigned)*/4 * 8 - 1);
                }
                update(node);
                ++oPath;
            } while ((--passes) != 0);
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
        return true;
    }

    public void remove(Node leaf) {
        removeLeaf(this, leaf);
        deleteNode(this, leaf);
        leaves--;
    }

    public void write(IWriter iwriter) {
        throw new UnsupportedOperationException();
    }

    public void clone(Dbvt dest) {
        clone(dest, null);
    }

    public void clone(Dbvt dest, IClone iclone) {
        throw new UnsupportedOperationException();
    }

    public static int countLeaves(Node node) {
        if (node.isInternal()) {
            return countLeaves(node.child0) + countLeaves(node.child1);
        } else {
            return 1;
        }
    }

    public static void extractLeaves(Node node, ObjectArrayList<Node> leaves) {
        if (node.isInternal()) {
            extractLeaves(node.child0, leaves);
            extractLeaves(node.child1, leaves);
        } else {
            leaves.add(node);
        }
    }

    public static void enumNodes(Node root, ICollide policy) {
        policy.process(root);
        if (root.isInternal()) {
            enumNodes(root.child0, policy);
            enumNodes(root.child1, policy);
        }
    }

    public static void enumLeaves(Node root, ICollide policy) {
        if (root.isInternal()) {
            enumLeaves(root.child0, policy);
            enumLeaves(root.child1, policy);
        } else {
            policy.process(root);
        }
    }

    public static void collideTT(Node root0, Node root1, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 != null && root1 != null) {
            ObjectArrayList<sStkNN> stack = new ObjectArrayList<sStkNN>(DOUBLE_STACK_SIZE);
            stack.add(new sStkNN(root0, root1));
            do {
                sStkNN p = stack.remove(stack.size() - 1);
                if (p.a == p.b) {
                    if (p.a.isInternal()) {
                        stack.add(new sStkNN(p.a.child0, p.a.child0));
                        stack.add(new sStkNN(p.a.child1, p.a.child1));
                        stack.add(new sStkNN(p.a.child0, p.a.child1));
                    }
                } else if (DbvtAabbMm.Intersect(p.a.volume, p.b.volume)) {
                    if (p.a.isInternal()) {
                        if (p.b.isInternal()) {
                            stack.add(new sStkNN(p.a.child0, p.b.child0));
                            stack.add(new sStkNN(p.a.child1, p.b.child0));
                            stack.add(new sStkNN(p.a.child0, p.b.child1));
                            stack.add(new sStkNN(p.a.child1, p.b.child1));
                        } else {
                            stack.add(new sStkNN(p.a.child0, p.b));
                            stack.add(new sStkNN(p.a.child1, p.b));
                        }
                    } else {
                        if (p.b.isInternal()) {
                            stack.add(new sStkNN(p.a, p.b.child0));
                            stack.add(new sStkNN(p.a, p.b.child1));
                        } else {
                            policy.process(p.a, p.b);
                        }
                    }
                }
            } while (!stack.isEmpty());
        }
    }

    public static void collideTT(Node root0, Node root1, Transform xform, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 != null && root1 != null) {
            ObjectArrayList<sStkNN> stack = new ObjectArrayList<sStkNN>(DOUBLE_STACK_SIZE);
            stack.add(new sStkNN(root0, root1));
            do {
                sStkNN p = stack.remove(stack.size() - 1);
                if (p.a == p.b) {
                    if (p.a.isInternal()) {
                        stack.add(new sStkNN(p.a.child0, p.a.child0));
                        stack.add(new sStkNN(p.a.child1, p.a.child1));
                        stack.add(new sStkNN(p.a.child0, p.a.child1));
                    }
                } else if (DbvtAabbMm.Intersect(p.a.volume, p.b.volume, xform)) {
                    if (p.a.isInternal()) {
                        if (p.b.isInternal()) {
                            stack.add(new sStkNN(p.a.child0, p.b.child0));
                            stack.add(new sStkNN(p.a.child1, p.b.child0));
                            stack.add(new sStkNN(p.a.child0, p.b.child1));
                            stack.add(new sStkNN(p.a.child1, p.b.child1));
                        } else {
                            stack.add(new sStkNN(p.a.child0, p.b));
                            stack.add(new sStkNN(p.a.child1, p.b));
                        }
                    } else {
                        if (p.b.isInternal()) {
                            stack.add(new sStkNN(p.a, p.b.child0));
                            stack.add(new sStkNN(p.a, p.b.child1));
                        } else {
                            policy.process(p.a, p.b);
                        }
                    }
                }
            } while (!stack.isEmpty());
        }
    }

    public static void collideTT(Node root0, Transform xform0, Node root1, Transform xform1, ICollide policy) {
        Transform xform = Stack.newTrans();
        xform.inverse(xform0);
        xform.mul(xform1);
        collideTT(root0, root1, xform, policy);
    }

    public static void collideTV(Node root, DbvtAabbMm volume, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            ObjectArrayList<Node> stack = new ObjectArrayList<Node>(SIMPLE_STACK_SIZE);
            stack.add(root);
            do {
                Node n = stack.remove(stack.size() - 1);
                if (DbvtAabbMm.Intersect(n.volume, volume)) {
                    if (n.isInternal()) {
                        stack.add(n.child0);
                        stack.add(n.child1);
                    } else {
                        policy.process(n);
                    }
                }
            } while (!stack.isEmpty());
        }
    }

    public static void collideRAY(Node root, Vector3d origin, Vector3d direction, ICollide policy) {
        if (root != null) {
            Vector3d normal = Stack.newVec();
            normal.normalize(direction);
            Vector3d invdir = Stack.newVec();
            invdir.set(1.0 / normal.x, 1.0 / normal.y, 1.0 / normal.z);
            int[] signs = new int[]{direction.x < 0 ? 1 : 0, direction.y < 0 ? 1 : 0, direction.z < 0 ? 1 : 0};
            ObjectArrayList<Node> stack = new ObjectArrayList<Node>(SIMPLE_STACK_SIZE);
            stack.add(root);
            do {
                Node node = stack.remove(stack.size() - 1);
                if (DbvtAabbMm.Intersect(node.volume, origin, invdir, signs)) {
                    if (node.isInternal()) {
                        stack.add(node.child0);
                        stack.add(node.child1);
                    } else {
                        policy.process(node);
                    }
                }
            } while (!stack.isEmpty());
        }
    }

    public static void collideKDOP(Node root, Vector3d[] normals, double[] offsets, int count, ICollide policy) {
        if (root != null) {
            int inside = (1 << count) - 1;
            ObjectArrayList<sStkNP> stack = new ObjectArrayList<sStkNP>(SIMPLE_STACK_SIZE);
            int[] signs = new int[4 * 8];
            assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
            for (int i = 0; i < count; ++i) {
                signs[i] = ((normals[i].x >= 0) ? 1 : 0) + ((normals[i].y >= 0) ? 2 : 0) + ((normals[i].z >= 0) ? 4 : 0);
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
                        stack.add(new sStkNP(se.node.child0, se.mask));
                        stack.add(new sStkNP(se.node.child1, se.mask));
                    } else {
                        if (policy.AllLeaves(se.node)) {
                            enumLeaves(se.node, policy);
                        }
                    }
                }
            } while (!stack.isEmpty());
        }
    }

    public static void collideOCL(Node root, Vector3d[] normals, double[] offsets, Vector3d sortaxis, int count, ICollide policy) {
        collideOCL(root, normals, offsets, sortaxis, count, policy, true);
    }

    public static void collideOCL(Node root, Vector3d[] normals, double[] offsets, Vector3d sortaxis, int count, ICollide policy, boolean fullsort) {
        if (root != null) {
            int srtsgns = (sortaxis.x >= 0 ? 1 : 0) + (sortaxis.y >= 0 ? 2 : 0) + (sortaxis.z >= 0 ? 4 : 0);
            int inside = (1 << count) - 1;
            ObjectArrayList<sStkNPS> stock = new ObjectArrayList<sStkNPS>();
            IntArrayList ifree = new IntArrayList();
            IntArrayList stack = new IntArrayList();
            int[] signs = new int[/*sizeof(unsigned)*8*/4 * 8];
            assert (count < (/*sizeof(signs)*/128 / /*sizeof(signs[0])*/ 4));
            for (int i = 0; i < count; i++) {
                signs[i] = ((normals[i].x >= 0) ? 1 : 0) + ((normals[i].y >= 0) ? 2 : 0) + ((normals[i].z >= 0) ? 4 : 0);
            }
            stack.add(allocate(ifree, stock, new sStkNPS(root, 0, root.volume.ProjectMinimum(sortaxis, srtsgns))));
            do {
                // JAVA NOTE: check
                int id = stack.remove(stack.size() - 1);
                sStkNPS se = stock.getQuick(id);
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
                        Node[] pns = new Node[]{se.node.child0, se.node.child1};
                        sStkNPS[] nes = new sStkNPS[]{new sStkNPS(pns[0], se.mask, pns[0].volume.ProjectMinimum(sortaxis, srtsgns)), new sStkNPS(pns[1], se.mask, pns[1].volume.ProjectMinimum(sortaxis, srtsgns))};
                        int q = nes[0].value < nes[1].value ? 1 : 0;
                        int j = stack.size();
                        if (fullsort && (j > 0)) {
                            /* Insert 0	*/
                            j = nearest(stack, stock, nes[q].value, 0, stack.size());
                            stack.add(0);
                            for (int k = stack.size() - 1; k > j; --k) {
                                stack.set(k, stack.get(k - 1));
                            }
                            stack.set(j, allocate(ifree, stock, nes[q]));
                            /* Insert 1	*/
                            j = nearest(stack, stock, nes[1 - q].value, j, stack.size());
                            stack.add(0);
                            for (int k = stack.size() - 1; k > j; --k) {
                                stack.set(k, stack.get(k - 1));
                            }
                            stack.set(j, allocate(ifree, stock, nes[1 - q]));
                        } else {
                            stack.add(allocate(ifree, stock, nes[q]));
                            stack.add(allocate(ifree, stock, nes[1 - q]));
                        }
                    } else {
                        policy.process(se.node, se.value);
                    }
                }
            } while (stack.size() != 0);
        }
    }

    public static void collideTU(Node root, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root != null) {
            ObjectArrayList<Node> stack = new ObjectArrayList<Node>(SIMPLE_STACK_SIZE);
            stack.add(root);
            do {
                Node n = stack.remove(stack.size() - 1);
                if (policy.Descent(n)) {
                    if (n.isInternal()) {
                        stack.add(n.child0);
                        stack.add(n.child1);
                    } else {
                        policy.process(n);
                    }
                }
            } while (!stack.isEmpty());
        }
    }

    public static int nearest(IntArrayList i, ObjectArrayList<sStkNPS> a, double v, int l, int h) {
        int m = 0;
        while (l < h) {
            m = (l + h) >> 1;
            if (a.getQuick(i.get(m)).value >= v) {
                l = m + 1;
            } else {
                h = m;
            }
        }
        return h;
    }

    public static int allocate(IntArrayList ifree, ObjectArrayList<sStkNPS> stock, sStkNPS value) {
        int i;
        if (ifree.size() > 0) {
            i = ifree.get(ifree.size() - 1);
            ifree.remove(ifree.size() - 1);
            stock.getQuick(i).set(value);
        } else {
            i = stock.size();
            stock.add(value);
        }
        return (i);
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static int indexOf(Node node) {
        return (node.parent.child1 == node) ? 1 : 0;
    }

    private static DbvtAabbMm merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm out) {
        DbvtAabbMm.Merge(a, b, out);
        return out;
    }

    // volume+edge lengths
    private static double size(DbvtAabbMm a) {
        Vector3d edges = a.Lengths(Stack.newVec());
        return (edges.x * edges.y * edges.z + edges.x + edges.y + edges.z);
    }

    private static void deleteNode(Dbvt pdbvt, Node node) {
        pdbvt.free = node;
    }

    private static void recurseDeleteNode(Dbvt pdbvt, Node node) {
        if (node.isBranch()) {
            recurseDeleteNode(pdbvt, node.child0);
            recurseDeleteNode(pdbvt, node.child1);
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
        node.child1 = null;
        return node;
    }

    private static void insertLeaf(Dbvt pdbvt, Node root, Node leaf) {
        if (pdbvt.root == null) {
            pdbvt.root = leaf;
            leaf.parent = null;
        } else {
            while (root.isBranch()) {
                if (DbvtAabbMm.Proximity(root.child0.volume, leaf.volume) < DbvtAabbMm.Proximity(root.child1.volume, leaf.volume)) {
                    root = root.child0;
                } else {
                    root = root.child1;
                }
            }
            Node prev = root.parent;
            Node node = createNode(pdbvt, prev, merge(leaf.volume, root.volume, new DbvtAabbMm()), null);
            if (prev != null) {
                if (indexOf(root) == 0) prev.child0 = node;
                else prev.child1 = node;
                node.child0 = root;
                root.parent = node;
                node.child1 = leaf;
                leaf.parent = node;
                do {
                    if (!prev.volume.Contain(node.volume)) {
                        DbvtAabbMm.Merge(prev.child0.volume, prev.child1.volume, prev.volume);
                    } else {
                        break;
                    }
                    node = prev;
                } while (null != (prev = node.parent));
            } else {
                node.child0 = root;
                root.parent = node;
                node.child1 = leaf;
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
            Node sibling = indexOf(leaf) == 0 ? parent.child1 : parent.child0;
            if (prev != null) {
                if (indexOf(parent) == 0) prev.child0 = sibling;
                else prev.child1 = sibling;
                sibling.parent = prev;
                deleteNode(pdbvt, parent);
                while (prev != null) {
                    DbvtAabbMm pb = prev.volume;
                    DbvtAabbMm.Merge(prev.child0.volume, prev.child1.volume, prev.volume);
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

    private static void fetchLeaves(Dbvt pdbvt, Node root, ObjectArrayList<Node> leaves) {
        fetchLeaves(pdbvt, root, leaves, -1);
    }

    private static void fetchLeaves(Dbvt pdbvt, Node root, ObjectArrayList<Node> leaves, int depth) {
        if (root.isInternal() && depth != 0) {
            fetchLeaves(pdbvt, root.child0, leaves, depth - 1);
            fetchLeaves(pdbvt, root.child1, leaves, depth - 1);
            deleteNode(pdbvt, root);
        } else {
            leaves.add(root);
        }
    }

    private static void split(ObjectArrayList<Node> leaves, ObjectArrayList<Node> left, ObjectArrayList<Node> right, Vector3d org, Vector3d axis) {
        Vector3d tmp = Stack.newVec();
        MiscUtil.resize(left, 0, Node.class);
        MiscUtil.resize(right, 0, Node.class);
        for (int i = 0, ni = leaves.size(); i < ni; i++) {
            leaves.getQuick(i).volume.Center(tmp);
            tmp.sub(org);
            if (axis.dot(tmp) < 0.0) {
                left.add(leaves.getQuick(i));
            } else {
                right.add(leaves.getQuick(i));
            }
        }
    }

    private static DbvtAabbMm bounds(ObjectArrayList<Node> leaves) {
        DbvtAabbMm volume = new DbvtAabbMm(leaves.getQuick(0).volume);
        for (int i = 1, ni = leaves.size(); i < ni; i++) {
            merge(volume, leaves.getQuick(i).volume, volume);
        }
        return volume;
    }

    private static void bottomUp(Dbvt pdbvt, ObjectArrayList<Node> leaves) {
        DbvtAabbMm tmpVolume = new DbvtAabbMm();
        while (leaves.size() > 1) {
            double minsize = BulletGlobals.SIMD_INFINITY;
            int[] minIdx = new int[]{-1, -1};
            for (int i = 0; i < leaves.size(); i++) {
                for (int j = i + 1; j < leaves.size(); j++) {
                    double sz = size(merge(leaves.getQuick(i).volume, leaves.getQuick(j).volume, tmpVolume));
                    if (sz < minsize) {
                        minsize = sz;
                        minIdx[0] = i;
                        minIdx[1] = j;
                    }
                }
            }
            Node n0 = leaves.getQuick(minIdx[0]);
            Node n1 = leaves.getQuick(minIdx[1]);
            Node p = createNode(pdbvt, null, merge(n0.volume, n1.volume, new DbvtAabbMm()), null);
            p.child0 = n0;
            p.child1 = n1;
            n0.parent = p;
            n1.parent = p;
            // JAVA NOTE: check
            leaves.setQuick(minIdx[0], p);
            Collections.swap(leaves, minIdx[1], leaves.size() - 1);
            leaves.removeQuick(leaves.size() - 1);
        }
    }

    private static final Vector3d[] axis = new Vector3d[]{new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, 0, 1)};

    private static Node topdown(Dbvt pdbvt, ObjectArrayList<Node> leaves, int bu_treshold) {
        if (leaves.size() > 1) {
            if (leaves.size() > bu_treshold) {
                DbvtAabbMm vol = bounds(leaves);
                Vector3d org = vol.Center(Stack.newVec());
                ObjectArrayList<Dbvt.Node> set0 = new ObjectArrayList<>();
                ObjectArrayList<Dbvt.Node> set1 = new ObjectArrayList<>();
                int bestaxis = -1;
                int bestmidp = leaves.size();
                int[][] splitcount = new int[/*3*/][/*2*/]{{0, 0}, {0, 0}, {0, 0}};

                Vector3d x = Stack.newVec();

                for (int i = 0; i < leaves.size(); i++) {
                    leaves.getQuick(i).volume.Center(x);
                    x.sub(org);
                    for (int j = 0; j < 3; j++) {
                        splitcount[j][x.dot(axis[j]) > 0.0 ? 1 : 0]++;
                    }
                }
                for (int i = 0; i < 3; i++) {
                    if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0)) {
                        int midp = Math.abs(splitcount[i][0] - splitcount[i][1]);
                        if (midp < bestmidp) {
                            bestaxis = i;
                            bestmidp = midp;
                        }
                    }
                }
                if (bestaxis >= 0) {
                    split(leaves, set0, set1, org, axis[bestaxis]);
                } else {
                    for (int i = 0, ni = leaves.size(); i < ni; i++) {
                        (((i & 1) == 0) ? set0 : set1).add(leaves.getQuick(i));
                    }
                }
                Node node = createNode(pdbvt, null, vol, null);
                node.child0 = topdown(pdbvt, set0, bu_treshold);
                node.child1 = topdown(pdbvt, set1, bu_treshold);
                node.child0.parent = node;
                node.child1.parent = node;
                return node;
            } else {
                bottomUp(pdbvt, leaves);
                return leaves.getQuick(0);
            }
        }
        return leaves.getQuick(0);
    }

    private static Node sort(Node n, Node[] r) {
        Node p = n.parent;
        assert (n.isInternal());
        // JAVA TODO: fix this
        if (p != null && p.hashCode() > n.hashCode()) {
            int i = indexOf(n);
            int j = 1 - i;
            Node s = j == 0 ? p.child0 : p.child1;
            Node q = p.parent;
            assert (n == (i == 0 ? p.child0 : p.child1));
            if (q != null) {
                if (indexOf(p) == 0) q.child0 = n;
                else q.child1 = n;
            } else {
                r[0] = n;
            }
            s.parent = n;
            p.parent = n;
            n.parent = q;
            p.child0 = n.child0;
            p.child1 = n.child1;
            n.child0.parent = p;
            n.child1.parent = p;

            if (i == 0) {
                n.child0 = p;
                n.child1 = s;
            } else {
                n.child0 = s;
                n.child1 = p;
            }

            DbvtAabbMm.swap(p.volume, n.volume);
            return p;
        }
        return n;
    }

    private static Node walkUp(Node n, int count) {
        while (n != null && (count--) != 0) {
            n = n.parent;
        }
        return n;
    }

    /// /////////////////////////////////////////////////////////////////////////

    public static class Node {
        public final DbvtAabbMm volume = new DbvtAabbMm();
        public Node parent;
        public Node child0;
        public Node child1;
        public Object data;

        public boolean isLeaf() {
            return child1 == null;
        }

        public boolean isBranch() {
            return child1 != null;
        }

        public boolean isInternal() {
            return !isLeaf();
        }
    }

    /**
     * Stack element
     */
    public static class sStkNN {
        public Node a;
        public Node b;

        public sStkNN(Node na, Node nb) {
            a = na;
            b = nb;
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

    public static class sStkCLN {
        public Node node;
        public Node parent;

        public sStkCLN(Node n, Node p) {
            node = n;
            parent = p;
        }
    }

    public static class ICollide {
        public void process(Node n1, Node n2) {
        }

        public void process(Node n) {
        }

        public void process(Node n, double f) {
            process(n);
        }

        public boolean Descent(Node n) {
            return true;
        }

        public boolean AllLeaves(Node n) {
            return true;
        }
    }

    public static abstract class IWriter {
        public abstract void Prepare(Node root, int numnodes);

        public abstract void WriteNode(Node n, int index, int parent, int child0, int child1);

        public abstract void WriteLeaf(Node n, int index, int parent);
    }

    public static class IClone {
        public void CloneLeaf(Node n) {
        }
    }

}
