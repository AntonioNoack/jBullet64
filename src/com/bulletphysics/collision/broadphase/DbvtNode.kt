package com.bulletphysics.collision.broadphase;

public class DbvtNode {
    public final DbvtAabbMm volume = new DbvtAabbMm();
    public DbvtNode parent;
    public DbvtNode child0;
    public DbvtNode child1;
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