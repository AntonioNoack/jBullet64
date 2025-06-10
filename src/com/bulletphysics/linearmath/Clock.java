package com.bulletphysics.linearmath;

/**
 * Clock is a portable basic clock that measures accurate time in seconds, use for profiling.
 * 
 * @author jezek2
 */
public class Clock {
	
	private long startTime;

	/**
	 * Creates a new clock and resets it.
	 */
	public Clock() {
		reset();
	}

	/**
	 * Resets clock by setting start time to current.
	 */
	public void reset() {
		startTime = System.nanoTime();
	}

	/**
	 * Returns the time in microseconds since the last call to reset or since the Clock was created.
	 */
	public long getTimeNanos() {
		return System.nanoTime() - startTime;
	}
	
}
