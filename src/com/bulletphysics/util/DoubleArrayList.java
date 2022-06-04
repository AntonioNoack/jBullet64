package com.bulletphysics.util;

/**
 *
 * @author jezek2
 */
public class DoubleArrayList {

	private double[] array = new double[16];
	private int size;
	
	public void add(double value) {
		if (size == array.length) expand();
		array[size++] = value;
	}
	
	private void expand() {
		double[] newArray = new double[array.length << 1];
		System.arraycopy(array, 0, newArray, 0, array.length);
		array = newArray;
	}

	public double remove(int index) {
		double old = array[index];
		System.arraycopy(array, index+1, array, index, size - index - 1);
		size--;
		return old;
	}

	public void setSize(int newSize) {
		if (array.length < newSize) {
			int newSize2 = array.length;
			while (newSize2 < newSize) {
				newSize2 <<= 1;
			}
			double[] newArray = new double[newSize2];
			System.arraycopy(array, 0, newArray, 0, array.length);
			array = newArray;
		}
		for (int i = size; i < newSize; i++) {
			array[i] = 0;
		}
		size = newSize;
	}


	public double get(int index) {
		return array[index];
	}

	public void set(int index, double value) {
		array[index] = value;
	}

	public int size() {
		return size;
	}

}
