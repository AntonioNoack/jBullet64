package com.bulletphysics.collision.broadphase;

/**
 * 
 * @author jezek2
 */
public enum DispatchFunc {

	DISPATCH_DISCRETE(1),
	DISPATCH_CONTINUOUS(2);
	
	private final int value;
	
	private DispatchFunc(int value) {
		this.value = value;
	}

	public int getValue() {
		return value;
	}
	
}
