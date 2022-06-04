package com.bulletphysics.extras.gimpact;

/**
 * Overlapping pair.
 * 
 * @author jezek2
 */
class Pair {

	public int index1;
	public int index2;

	public Pair() {
	}

	public Pair(int index1, int index2) {
		this.index1 = index1;
		this.index2 = index2;
	}

	public Pair(Pair p) {
		index1 = p.index1;
		index2 = p.index2;
	}

}
