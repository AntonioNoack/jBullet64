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

package com.bulletphysics.collision.shapes;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * ConvexInternalShape is an internal base class, shared by most convex shape implementations.
 * 
 * @author jezek2
 */
public abstract class ConvexInternalShape extends ConvexShape {

	// local scaling. collisionMargin is not scaled !
	protected final Vector3d localScaling = new Vector3d(1.0, 1.0, 1.0);
	protected final Vector3d implicitShapeDimensions = new Vector3d();
	protected double collisionMargin = BulletGlobals.CONVEX_DISTANCE_MARGIN;

	/**
	 * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
	 */
	@Override
	public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		getAabbSlow(t, aabbMin, aabbMax);
	}
	
	@Override
	public void getAabbSlow(Transform trans, Vector3d minAabb, Vector3d maxAabb) {
		double margin = getMargin();
		Vector3d vec = Stack.newVec();
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();
		
		for (int i=0;i<3;i++)
		{
			vec.set(0.0, 0.0, 0.0);
			VectorUtil.setCoord(vec, i, 1f);

			MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
			localGetSupportingVertex(tmp1, tmp2);
			
			trans.transform(tmp2);

			VectorUtil.setCoord(maxAabb, i, VectorUtil.getCoord(tmp2, i) + margin);

			VectorUtil.setCoord(vec, i, -1f);

			MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
			localGetSupportingVertex(tmp1, tmp2);
			trans.transform(tmp2);

			VectorUtil.setCoord(minAabb, i, VectorUtil.getCoord(tmp2, i) - margin);
		}

		Stack.subVec(3);

	}

	@Override
	public Vector3d localGetSupportingVertex(Vector3d vec, Vector3d out) {
		Vector3d supVertex = localGetSupportingVertexWithoutMargin(vec, out);

		if (getMargin() != 0f) {
			Vector3d vecnorm = Stack.newVec(vec);
			if (vecnorm.lengthSquared() < (BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
				vecnorm.set(-1f, -1f, -1f);
			}
			vecnorm.normalize();
			supVertex.scaleAdd(getMargin(), vecnorm, supVertex);
			Stack.subVec(1);
		}
		return out;
	}
	
	public void setLocalScaling(Vector3d scaling) {
		localScaling.absolute(scaling);
	}
	
	public Vector3d getLocalScaling(Vector3d out) {
		out.set(localScaling);
		return out;
	}

	public double getMargin() {
		return collisionMargin;
	}

	public void setMargin(double margin) {
		this.collisionMargin = margin;
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return 0;
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vector3d penetrationVector) {
		throw new InternalError();
	}
	
}
