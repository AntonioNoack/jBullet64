package com.bulletphysics.collision.broadphase;

/**
 * Common collision filter groups.
 * 
 * @author jezek2
 */
public class CollisionFilterGroups {

	public static final short DEFAULT_FILTER   = 1;
	public static final short STATIC_FILTER    = 2;
	public static final short KINEMATIC_FILTER = 4;
	public static final short DEBRIS_FILTER    = 8;
	public static final short SENSOR_TRIGGER   = 16;
	public static final short CHARACTER_FILTER = 32;
	public static final short ALL_FILTER       = -1; // all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
	
}
