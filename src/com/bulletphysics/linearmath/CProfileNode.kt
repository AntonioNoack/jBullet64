
package com.bulletphysics.linearmath;

import com.bulletphysics.BulletStats;

import java.util.Objects;

/***************************************************************************************************
 * <p>
 * Real-Time Hierarchical Profiling for Game Programming Gems 3
 * <p>
 * by Greg Hjelstrom & Byon Garrabrant
 * <p>
 ***************************************************************************************************
 *
 * A node in the Profile Hierarchy Tree.
 *
 * @author jezek2
 */
public class CProfileNode {

    public String name;
    public int totalCalls;
    public double totalTime;

    private long startTime;
    private int recursionCounter;

    private final CProfileNode parent;
    private CProfileNode child;
    private CProfileNode sibling;

    public CProfileNode(String name, CProfileNode parent) {
        this.name = name;
        this.totalCalls = 0;
        this.totalTime = 0;
        this.startTime = 0;
        this.recursionCounter = 0;
        this.parent = parent;
        this.child = null;
        this.sibling = null;

        reset();
    }

    public CProfileNode getSubNode(String name) {
        // Try to find this sub node
        CProfileNode child = this.child;
        while (child != null) {
            if (Objects.equals(child.name, name)) {
                return child;
            }
            child = child.sibling;
        }

        // We didn't find it, so add it

        CProfileNode node = new CProfileNode(name, this);
        node.sibling = this.child;
        this.child = node;
        return node;
    }

    public CProfileNode getParent() {
        return parent;
    }

    public CProfileNode getSibling() {
        return sibling;
    }

    public CProfileNode getChild() {
        return child;
    }

    public void cleanupMemory() {
        child = null;
        sibling = null;
    }

    public void reset() {
        totalCalls = 0;
        totalTime = 0.0;
        BulletStats.profileClock.reset();

        if (child != null) {
            child.reset();
        }
        if (sibling != null) {
            sibling.reset();
        }
    }

    public void call() {
        totalCalls++;
        if (recursionCounter++ == 0) {
            startTime = BulletStats.profileGetTicks();
        }
    }

    public boolean Return() {
        if (--recursionCounter == 0 && totalCalls != 0) {
            long time = BulletStats.profileGetTicks() - startTime;
            totalTime += (double) time / BulletStats.profileGetTickRate();
        }
        return (recursionCounter == 0);
    }

}
