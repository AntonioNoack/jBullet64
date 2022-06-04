
// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

/**
 *
 * @author jezek2
 */
public class DbvtProxy extends BroadphaseProxy {

	public final DbvtAabbMm aabb = new DbvtAabbMm();
	public Dbvt.Node leaf;
	public final DbvtProxy[] links = new DbvtProxy[2];
	public int stage;

	public DbvtProxy(Object userPtr, short collisionFilterGroup, short collisionFilterMask) {
		super(userPtr, collisionFilterGroup, collisionFilterMask);
	}

}
