
/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/
package com.bulletphysics.linearmath;

import com.bulletphysics.BulletStats;

import java.util.Objects;

/**
 * Manager for the profile system.
 * 
 * @author jezek2
 */
public class CProfileManager {

	private static final CProfileNode root = new CProfileNode("Root", null);
	private static CProfileNode currentNode = root;
	private static int frameCounter = 0;
	private static long resetTime = 0;

	/**
	 * @param name must be {@link String#intern interned} String (not needed for String literals)
	 */
	public static void startProfile(String name) {
		if (!Objects.equals(name, currentNode.name)) {
			currentNode = currentNode.getSubNode(name);
		}

		currentNode.call();
	}
	
	public static void stopProfile() {
		// Return will indicate whether we should back up to our parent (we may
		// be profiling a recursive function)
		if (currentNode.Return()) {
			currentNode = currentNode.getParent();
		}
	}

	public static void cleanupMemory() {
		root.cleanupMemory();
	}

	public static void reset() {
		root.reset();
		root.call();
		frameCounter = 0;
		resetTime = BulletStats.profileGetTicks();
	}
	
	public static void incrementFrameCounter() {
		frameCounter++;
	}
	
	public static int getFrameCountSinceReset() {
		return frameCounter;
	}
	
	public static double getTimeSinceReset() {
		long time = BulletStats.profileGetTicks();
		time -= resetTime;
		return (double) time / BulletStats.profileGetTickRate();
	}

	public static CProfileIterator getIterator() {
		return new CProfileIterator(root);
	}
	
	public static void releaseIterator(CProfileIterator iterator) {
		/*delete ( iterator);*/
	}
	
}
