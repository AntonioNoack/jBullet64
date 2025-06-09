package com.bulletphysics.collision.broadphase;

/**
 * Stack element
 */
public class DbvtBranch {
    public DbvtNode a;
    public DbvtNode b;

    public DbvtBranch() {
    }

    public void set(DbvtNode na, DbvtNode nb) {
        a = na;
        b = nb;
    }
}