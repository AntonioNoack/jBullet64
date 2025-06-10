// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class Dbvt {

    public DbvtNode root = null;
    public DbvtNode free = null;
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

    public void optimizeIncremental(int passes) {
        if (passes < 0) {
            passes = leaves;
        }

        if (root != null && (passes > 0)) {
            DbvtNode[] root_ref = new DbvtNode[1];
            do {
                DbvtNode node = root;
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

    public DbvtNode insert(DbvtAabbMm box, Object data) {
        DbvtNode leaf = createNode(this, null, box, data);
        insertLeaf(this, root, leaf);
        leaves++;
        return leaf;
    }

    public void update(DbvtNode leaf) {
        update(leaf, -1);
    }

    public void update(DbvtNode leaf, int lookahead) {
        DbvtNode root = removeLeaf(this, leaf);
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

    public void update(DbvtNode leaf, DbvtAabbMm volume) {
        DbvtNode root = removeLeaf(this, leaf);
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

    public boolean update(DbvtNode leaf, DbvtAabbMm volume, Vector3d velocity, double margin) {
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

    public boolean update(DbvtNode leaf, DbvtAabbMm volume, Vector3d velocity) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        volume.SignedExpand(velocity);
        update(leaf, volume);
        return true;
    }

    public boolean update(DbvtNode leaf, DbvtAabbMm volume, double margin) {
        if (leaf.volume.Contain(volume)) {
            return false;
        }
        Vector3d tmp = Stack.newVec();
        tmp.set(margin, margin, margin);
        volume.Expand(tmp);
        update(leaf, volume);
        return true;
    }

    public void remove(DbvtNode leaf) {
        removeLeaf(this, leaf);
        deleteNode(this, leaf);
        leaves--;
    }

    private static void addBranch(ObjectArrayList<DbvtNode> remaining, DbvtNode na, DbvtNode nb) {
        // added in reverse, so they can be popped correctly
        remaining.add(nb);
        remaining.add(na);
    }

    public static void collideTT(DbvtNode root0, DbvtNode root1, ICollide policy) {
        //DBVT_CHECKTYPE
        if (root0 == null || root1 == null) return;

        ObjectArrayList<DbvtNode> remaining = Stack.newList();
        addBranch(remaining, root0, root1);
        while (!remaining.isEmpty()) {
            DbvtNode pa = remaining.removeLast();
            DbvtNode pb = remaining.removeLast();
            if (pa == pb) {
                if (pa.isInternal()) {
                    addBranch(remaining, pa.child0, pa.child0);
                    addBranch(remaining, pa.child1, pa.child1);
                    addBranch(remaining, pa.child0, pa.child1);
                }
            } else if (DbvtAabbMm.Intersect(pa.volume, pb.volume)) {
                if (pa.isInternal()) {
                    if (pb.isInternal()) {
                        addBranch(remaining, pa.child0, pb.child0);
                        addBranch(remaining, pa.child1, pb.child0);
                        addBranch(remaining, pa.child0, pb.child1);
                        addBranch(remaining, pa.child1, pb.child1);
                    } else {
                        addBranch(remaining, pa.child0, pb);
                        addBranch(remaining, pa.child1, pb);
                    }
                } else {
                    if (pb.isInternal()) {
                        addBranch(remaining, pa, pb.child0);
                        addBranch(remaining, pa, pb.child1);
                    } else {
                        policy.process(pa, pb);
                    }
                }
            }
        }
        Stack.subList(1);
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static int indexOf(DbvtNode node) {
        return (node.parent.child1 == node) ? 1 : 0;
    }

    private static DbvtAabbMm merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm out) {
        DbvtAabbMm.Merge(a, b, out);
        return out;
    }

    private static void deleteNode(Dbvt pdbvt, DbvtNode node) {
        pdbvt.free = node;
    }

    private static void recurseDeleteNode(Dbvt pdbvt, DbvtNode node) {
        if (node.isBranch()) {
            recurseDeleteNode(pdbvt, node.child0);
            recurseDeleteNode(pdbvt, node.child1);
        }
        if (node == pdbvt.root) {
            pdbvt.root = null;
        }
        deleteNode(pdbvt, node);
    }

    private static DbvtNode createNode(Dbvt pdbvt, DbvtNode parent, DbvtAabbMm volume, Object data) {
        DbvtNode node;
        if (pdbvt.free != null) {
            node = pdbvt.free;
            pdbvt.free = null;
        } else {
            node = new DbvtNode();
        }
        node.parent = parent;
        node.volume.set(volume);
        node.data = data;
        node.child1 = null;
        return node;
    }

    private static void insertLeaf(Dbvt pdbvt, DbvtNode root, DbvtNode leaf) {
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
            DbvtNode prev = root.parent;
            DbvtAabbMm volume = merge(leaf.volume, root.volume, Stack.newDbvtAabbMm());
            DbvtNode node = createNode(pdbvt, prev, volume, null);
            Stack.subDbvtAabbMm(1); // volume
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

    private static DbvtNode removeLeaf(Dbvt pdbvt, DbvtNode leaf) {
        if (leaf == pdbvt.root) {
            pdbvt.root = null;
            return null;
        } else {
            DbvtNode parent = leaf.parent;
            DbvtNode prev = parent.parent;
            DbvtNode sibling = indexOf(leaf) == 0 ? parent.child1 : parent.child0;
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

    private static DbvtNode sort(DbvtNode n, DbvtNode[] r) {
        DbvtNode p = n.parent;
        assert (n.isInternal());
        // JAVA TODO: fix this
        if (p != null && p.hashCode() > n.hashCode()) {
            int i = indexOf(n);
            int j = 1 - i;
            DbvtNode s = j == 0 ? p.child0 : p.child1;
            DbvtNode q = p.parent;
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

    /// /////////////////////////////////////////////////////////////////////////

    public static class ICollide {
        public void process(DbvtNode n1, DbvtNode n2) {
        }
    }
}
