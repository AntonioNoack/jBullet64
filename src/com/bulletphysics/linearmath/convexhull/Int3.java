package com.bulletphysics.linearmath.convexhull;

/**
 *
 * @author jezek2
 */
class Int3 {

	public int x, y, z;

	public Int3() {
	}

	public Int3(int x, int y, int z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public Int3(Int3 i) {
		x = i.x;
		y = i.y;
		z = i.z;
	}
	
	public void set(int x, int y, int z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public void set(Int3 i) {
		x = i.x;
		y = i.y;
		z = i.z;
	}
	
	public int getCoord(int coord) {
		switch (coord) {
			case 0: return x;
			case 1: return y;
			default: return z;
		}
	}

	public void setCoord(int coord, int value) {
		switch (coord) {
			case 0: x = value; break;
			case 1: y = value; break;
			case 2: z = value; break;
		}
	}
	
	public boolean equals(Int3 i) {
		return (x == i.x && y == i.y && z == i.z);
	}

}
