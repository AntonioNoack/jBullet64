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

    private static int er = -1;

    private static final IntRef erRef = new IntRef() {
        @Override
        public int get() {
            return er;
        }

        @Override
        public void set(int value) {
            er = value;
        }
    };

    public IntRef neib(int a, int b) {
        for (int i = 0; i < 3; i++) {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;

            if (getCoord(i) == a && getCoord(i1) == b) {
                return n.getRef(i2);
            }
            if (getCoord(i) == b && getCoord(i1) == a) {
                return n.getRef(i2);
            }
        }
        assert (false);
        return erRef;
    }

}
