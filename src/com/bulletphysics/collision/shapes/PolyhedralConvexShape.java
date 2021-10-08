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

import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 * PolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
 * 
 * @author jezek2
 */
public abstract class PolyhedralConvexShape extends ConvexInternalShape {

	private static final Vector3d[] _directions = new Vector3d[] {
		new Vector3d( 1.0,  0.0,  0.0),
		new Vector3d( 0.0,  1.0,  0.0),
		new Vector3d( 0.0,  0.0,  1.0),
		new Vector3d(-1f,  0.0,  0.0),
		new Vector3d( 0.0, -1f,  0.0),
		new Vector3d( 0.0,  0.0, -1f)
	};

	private static final Vector3d[] _supporting = new Vector3d[] {
		new Vector3d(0.0, 0.0, 0.0),
		new Vector3d(0.0, 0.0, 0.0),
		new Vector3d(0.0, 0.0, 0.0),
		new Vector3d(0.0, 0.0, 0.0),
		new Vector3d(0.0, 0.0, 0.0),
		new Vector3d(0.0, 0.0, 0.0)
	};
	
	protected final Vector3d localAabbMin = new Vector3d(1.0, 1.0, 1.0);
	protected final Vector3d localAabbMax = new Vector3d(-1f, -1f, -1f);
	protected boolean isLocalAabbValid = false;

//	/** optional Hull is for optional Separating Axis Test Hull collision detection, see Hull.cpp */
//	public Hull optionalHull = null;
	
	@Override
	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d vec0, Vector3d out) {
		int i;
		out.set(0.0, 0.0, 0.0);

		double maxDot = -1e300;

		Vector3d vec = Stack.newVec(vec0);
		double lenSqr = vec.lengthSquared();
		if (lenSqr < 0.0001f) {
			vec.set(1.0, 0.0, 0.0);
		}
		else {
			double rlen = 1.0 / Math.sqrt(lenSqr);
			vec.scale(rlen);
		}

		Vector3d vtx = Stack.newVec();
		double newDot;

		for (i = 0; i < getNumVertices(); i++) {
			getVertex(i, vtx);
			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				// supVec = vtx;
			}
		}

		Stack.subVec(2);

		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors) {
		int i;

		Vector3d vtx = Stack.newVec();
		double newDot;

		// JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
		// TODO: optimize it
		double[] wcoords = new double[numVectors];

		for (i = 0; i < numVectors; i++) {
			// TODO: used w in vector3:
			//supportVerticesOut[i].w = -1e300;
			wcoords[i] = -1e300;
		}

		for (int j = 0; j < numVectors; j++) {
			Vector3d vec = vectors[j];

			for (i = 0; i < getNumVertices(); i++) {
				getVertex(i, vtx);
				newDot = vec.dot(vtx);
				//if (newDot > supportVerticesOut[j].w)
				if (newDot > wcoords[j]) {
					//WARNING: don't swap next lines, the w component would get overwritten!
					supportVerticesOut[j].set(vtx);
					//supportVerticesOut[j].w = newDot;
					wcoords[j] = newDot;
				}
			}
		}
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		// not yet, return box inertia

		double margin = getMargin();

		Transform ident = Stack.newTrans();
		ident.setIdentity();
		Vector3d aabbMin = Stack.newVec(), aabbMax = Stack.newVec();
		getAabb(ident, aabbMin, aabbMax);

		Vector3d halfExtents = Stack.newVec();
		halfExtents.sub(aabbMax, aabbMin);
		halfExtents.scale(0.5);

		double lx = 2f * (halfExtents.x + margin);
		double ly = 2f * (halfExtents.y + margin);
		double lz = 2f * (halfExtents.z + margin);
		double x2 = lx * lx;
		double y2 = ly * ly;
		double z2 = lz * lz;
		double scaledmass = mass * 0.08333333f;

		inertia.set(y2 + z2, x2 + z2, x2 + y2);
		inertia.scale(scaledmass);
	}

	private void getNonvirtualAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax, double margin) {
		// lazy evaluation of local aabb
		assert (isLocalAabbValid);

		AabbUtil2.transformAabb(localAabbMin, localAabbMax, margin, trans, aabbMin, aabbMax);
	}
	
	@Override
	public void getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
		getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
	}

	protected final void _PolyhedralConvexShape_getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
		getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
	}

	public void recalcLocalAabb() {
		isLocalAabbValid = true;

		//#if 1

		batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

		for (int i=0; i<3; i++) {
			VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(_supporting[i], i) + collisionMargin);
			VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(_supporting[i + 3], i) - collisionMargin);
		}
		
		//#else
		//for (int i=0; i<3; i++) {
		//	Vector3d vec = new Vector3d();
		//	vec.set(0.0, 0.0, 0.0);
		//	VectorUtil.setCoord(vec, i, 1.0);
		//	Vector3d tmp = localGetSupportingVertex(vec, new Vector3d());
		//	VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmp, i) + collisionMargin);
		//	VectorUtil.setCoord(vec, i, -1f);
		//	localGetSupportingVertex(vec, tmp);
		//	VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmp, i) - collisionMargin);
		//}
		//#endif
	}

	@Override
	public void setLocalScaling(Vector3d scaling) {
		super.setLocalScaling(scaling);
		recalcLocalAabb();
	}

	public abstract int getNumVertices();

	public abstract int getNumEdges();

	public abstract void getEdge(int i, Vector3d pa, Vector3d pb);

	public abstract void getVertex(int i, Vector3d vtx);

	public abstract int getNumPlanes();

	public abstract void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i);
	
//	public abstract  int getIndex(int i) const = 0 ; 
	
	public abstract boolean isInside(Vector3d pt, double tolerance);
	
}
