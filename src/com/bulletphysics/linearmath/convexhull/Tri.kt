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

    public int neibGet(int a, int b) {
        int x = this.x, y = this.y, z = this.z;
        if ((x == a && y == b) || (x == b && y == a)) {
            return n.z;
        } else if ((y == a && z == b) || (y == b && z == a)) {
            return n.x;
        } else {
            return n.y;
        }
    }

    public void neibSet(int a, int b, int value) {
        int x = this.x, y = this.y, z = this.z;
        if ((x == a && y == b) || (x == b && y == a)) {
            n.z = value;
        } else if ((y == a && z == b) || (y == b && z == a)) {
            n.x = value;
        } else {
            n.y = value;
        }
    }

}
