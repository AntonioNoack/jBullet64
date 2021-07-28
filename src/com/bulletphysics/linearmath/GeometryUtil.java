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

package com.bulletphysics.linearmath;

import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 * GeometryUtil helper class provides a few methods to convert between plane
 * equations and vertices.
 * 
 * @author jezek2
 */
public class GeometryUtil {

	public static boolean isPointInsidePlanes(ObjectArrayList<Vector4d> planeEquations, Vector3d point, double margin) {
		int numbrushes = planeEquations.size();
		for (int i = 0; i < numbrushes; i++) {
			Vector4d N1 = planeEquations.getQuick(i);
			double dist = VectorUtil.dot3(N1, point) + N1.w - margin;
			if (dist > 0f) {
				return false;
			}
		}
		return true;
	}
	
	public static boolean areVerticesBehindPlane(Vector4d planeNormal, ObjectArrayList<Vector3d> vertices, double margin) {
		int numvertices = vertices.size();
		for (int i = 0; i < numvertices; i++) {
			Vector3d N1 = vertices.getQuick(i);
			double dist = VectorUtil.dot3(planeNormal, N1) + planeNormal.w - margin;
			if (dist > 0f) {
				return false;
			}
		}
		return true;
	}
	
	private static boolean notExist(Vector4d planeEquation, ObjectArrayList<Vector4d> planeEquations) {
		int numbrushes = planeEquations.size();
		for (int i = 0; i < numbrushes; i++) {
			Vector4d N1 = planeEquations.getQuick(i);
			if (VectorUtil.dot3(planeEquation, N1) > 0.999f) {
				return false;
			}
		}
		return true;
	}

	public static void getPlaneEquationsFromVertices(ObjectArrayList<Vector3d> vertices, ObjectArrayList<Vector4d> planeEquationsOut) {
		Vector4d planeEquation = new Vector4d();
		Vector3d edge0 = new Vector3d(), edge1 = new Vector3d();
		Vector3d tmp = Stack.newVec();

		int numvertices = vertices.size();
		// brute force:
		for (int i = 0; i < numvertices; i++) {
			Vector3d N1 = vertices.getQuick(i);

			for (int j = i + 1; j < numvertices; j++) {
				Vector3d N2 = vertices.getQuick(j);

				for (int k = j + 1; k < numvertices; k++) {
					Vector3d N3 = vertices.getQuick(k);

					edge0.sub(N2, N1);
					edge1.sub(N3, N1);
					double normalSign = 1f;
					for (int ww = 0; ww < 2; ww++) {
						tmp.cross(edge0, edge1);
						planeEquation.x = normalSign * tmp.x;
						planeEquation.y = normalSign * tmp.y;
						planeEquation.z = normalSign * tmp.z;

						if (VectorUtil.lengthSquared3(planeEquation) > 0.0001f) {
							VectorUtil.normalize3(planeEquation);
							if (notExist(planeEquation, planeEquationsOut)) {
								planeEquation.w = -VectorUtil.dot3(planeEquation, N1);

								// check if inside, and replace supportingVertexOut if needed
								if (areVerticesBehindPlane(planeEquation, vertices, 0.01f)) {
									planeEquationsOut.add(new Vector4d(planeEquation));
								}
							}
						}
						normalSign = -1f;
					}
				}
			}
		}
	}
	
	public static void getVerticesFromPlaneEquations(ObjectArrayList<Vector4d> planeEquations, ObjectArrayList<Vector3d> verticesOut) {
		Vector3d n2n3 = new Vector3d();
		Vector3d n3n1 = new Vector3d();
		Vector3d n1n2 = new Vector3d();
		Vector3d potentialVertex = new Vector3d();

		int numbrushes = planeEquations.size();
		// brute force:
		for (int i = 0; i < numbrushes; i++) {
			Vector4d N1 = planeEquations.getQuick(i);

			for (int j = i + 1; j < numbrushes; j++) {
				Vector4d N2 = planeEquations.getQuick(j);

				for (int k = j + 1; k < numbrushes; k++) {
					Vector4d N3 = planeEquations.getQuick(k);

					VectorUtil.cross3(n2n3, N2, N3);
					VectorUtil.cross3(n3n1, N3, N1);
					VectorUtil.cross3(n1n2, N1, N2);

					if ((n2n3.lengthSquared() > 0.0001f) &&
							(n3n1.lengthSquared() > 0.0001f) &&
							(n1n2.lengthSquared() > 0.0001f)) {
						// point P out of 3 plane equations:

						// 	     d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
						// P =  -------------------------------------------------------------------------  
						//    N1 . ( N2 * N3 )  

						double quotient = VectorUtil.dot3(N1, n2n3);
						if (Math.abs(quotient) > 0.000001f) {
							quotient = -1f / quotient;
							n2n3.scale(N1.w);
							n3n1.scale(N2.w);
							n1n2.scale(N3.w);
							potentialVertex.set(n2n3);
							potentialVertex.add(n3n1);
							potentialVertex.add(n1n2);
							potentialVertex.scale(quotient);

							// check if inside, and replace supportingVertexOut if needed
							if (isPointInsidePlanes(planeEquations, potentialVertex, 0.01f)) {
								verticesOut.add(new Vector3d(potentialVertex));
							}
						}
					}
				}
			}
		}
	}
	
}
