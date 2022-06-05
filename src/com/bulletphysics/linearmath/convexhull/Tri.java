package com.bulletphysics.linearmath.convexhull;

/**
 * @author jezek2
 */
class Tri extends Int3 {

    public Int3 n = new Int3();
    public int id;
    public int maxValue;
    public double rise;

    public Tri(int a, int b, int c) {
        super(a, b, c);
        n.set(-1, -1, -1);
        maxValue = -1;
        rise = 0.0;
    }

    public int getNeighbor(int a, int b) {
        // find value which is neither a nor b
        if (x == a) {// yz remain
            return y == b ? n.z : n.y;
        } else if (y == a) {// xz remain
            return x == b ? n.z : n.x;
        } else { // z == a; xy remain
            return x == b ? n.y : n.x;
        }
    }

    public void setNeighbor(int a, int b, int value) {
        // set value which is neither a nor b
        if (x == a) {// yz remain
            if (y == b) n.z = value;
            else n.y = value;
        } else if (y == a) {// xz remain
            if (x == b) n.z = value;
            else n.x = value;
        } else { // z == a; xy remain
            if (x == b) n.y = value;
            else n.x = value;
        }
    }

}
