package com.bulletphysics.linearmath;

import com.bulletphysics.util.DoubleArrayList;
import com.bulletphysics.util.IntArrayList;

import java.util.Comparator;
import java.util.List;
import java.util.function.Predicate;

/**
 * Miscellaneous utility functions.
 *
 * @author jezek2
 */
public class MiscUtil {

	/**
	 * Resizes list to exact size
	 */
	public static void resize(IntArrayList list, int size) {
		list.setSize(size);
	}

	/**
	 * Resizes list to exact size
	 */
	public static void resize(DoubleArrayList list, int size) {
		list.setSize(size);
	}

	/**
	 * Resizes list to exact size, filling with new instances of given class type
	 * when expanding.
	 */
	public static <T> void resize(List<T> list, int size, Class<T> valueCls) {
		try {
			if (valueCls != null) {
				while (list.size() < size) {
					list.add(valueCls.newInstance());
				}
			} else {
				while (list.size() < size) {
					list.add(null);
				}
			}
			while (list.size() > size) {
				list.remove(list.size() - 1);
			}
		} catch (IllegalAccessException | InstantiationException e) {
			throw new IllegalStateException(e);
		}
	}

	public static double clamp(double a, double min, double max) {
		return a < min ? min : (max < a ? max : a);
	}

	/**
	 * Sorts list.
	 */
	public static <T> void sort(List<T> list, Comparator<T> comparator) {
		// don't sort 0 or 1 elements
		list.sort(comparator);
	}

	/**
	 * Sorts list.
	 */
	public static <T> void partition(List<T> list, Predicate<T> predicate) {
		int length = list.size();
		for (int i = 0; i < length; ) {
			T element = list.get(i);
			if (predicate.test(element)) {
				// place at the end
				length--;
				T elementFromEnd = list.get(length);
				list.set(length, element);
				list.set(i, elementFromEnd);
			} else {
				// leave it where it is
				i++;
			}
		}
	}

}
