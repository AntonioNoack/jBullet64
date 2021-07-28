/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.collision.narrowphase;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
public class PointCollector extends DiscreteCollisionDetectorInterface.Result {

	public final Vector3d normalOnBInWorld = new Vector3d();
	public final Vector3d pointInWorld = new Vector3d();
	public double distance = 1e300; // negative means penetration

	public boolean hasResult = false;
	
	public void setShapeIdentifiers(int partId0, int index0, int partId1, int index1) {
		// ??
	}

	public void addContactPoint(Vector3d normalOnBInWorld, Vector3d pointInWorld, double depth) {
		if (depth < distance) {
			hasResult = true;
			this.normalOnBInWorld.set(normalOnBInWorld);
			this.pointInWorld.set(pointInWorld);
			// negative means penetration
			distance = depth;
		}
	}

}
