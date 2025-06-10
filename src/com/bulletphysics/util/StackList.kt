package com.bulletphysics.util;

/**
 * Stack-based object pool, see the example for usage. You must use the {@link #returning}
 * method for returning stack-allocated instance.<p>
 * 
 * Example code:
 * 
 * <pre>
 * StackList&lt;Vector3d&gt; vectors;
 * ...
 * 
 * vectors.push();
 * try {
 *     Vector3d vec = vectors.get();
 *     ...
 *     return vectors.returning(vec);
 * }
 * finally {
 *     vectors.pop();
 * }
 * </pre>
 * 
 * @author jezek2
 */
public abstract class StackList<T> {

	private final ObjectArrayList<T> list = new ObjectArrayList<>();

	private final int[] stack = new int[512];
	private int stackCount = 0;
	
	private int pos = 0;

	protected StackList() {
	}
	
	/**
	 * Pushes the stack.
	 */
	public final void push() {
		stack[stackCount++] = pos;
	}

	/**
	 * Pops the stack.
	 */
	public final void pop() {
		pos = stack[--stackCount];
	}
	
	/**
	 * Returns instance from stack pool, or create one if not present. The returned
	 * instance will be automatically reused when {@link #pop} is called.
	 * 
	 * @return instance
	 */
	public T get() {
		if (pos == list.size()) {
			expand();
		}
		return list.getQuick(pos++);
	}

	/**
	 * Creates a new instance of type.
	 * 
	 * @return instance
	 */
	protected abstract T create();

	private void expand() {
		list.add(create());
	}
	
}
