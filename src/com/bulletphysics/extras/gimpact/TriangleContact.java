package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectArrayList;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 *
 * @author jezek2
 */
public class TriangleContact {
	
	private final ArrayPool<int[]> intArrays = ArrayPool.get(int.class);
	
	public static final int MAX_TRI_CLIPPING = 16;

    public double penetrationDepth;
    public int pointCount;
    public final Vector4d separatingNormal = new Vector4d();
    public Vector3d[] points = new Vector3d[MAX_TRI_CLIPPING];

	public TriangleContact() {
		for (int i=0; i<points.length; i++) {
			points[i] = new Vector3d();
		}
	}

	@SuppressWarnings({"unused", "CopyConstructorMissesField"})
    public TriangleContact(TriangleContact other) {
		copyFrom(other);
	}

	public void set(TriangleContact other) {
		copyFrom(other);
	}
	
	public void copyFrom(TriangleContact other) {
		penetrationDepth = other.penetrationDepth;
		separatingNormal.set(other.separatingNormal);
		pointCount = other.pointCount;
		int i = pointCount;
		while ((i--) != 0) {
			points[i].set(other.points[i]);
		}
	}
	
	/**
	 * Classify points that are closer.
	 */
	public void mergePoints(Vector4d plane, double margin, Vector3d[] points, int point_count) {
		this.pointCount = 0;
		penetrationDepth = -1000.0;

		int[] pointIndices = intArrays.getFixed(MAX_TRI_CLIPPING);

		for (int k = 0; k < point_count; k++) {
			double dist = -ClipPolygon.distancePointPlane(plane, points[k]) + margin;

			if (dist >= 0.0) {
				if (dist > penetrationDepth) {
					penetrationDepth = dist;
					pointIndices[0] = k;
					this.pointCount = 1;
				}
				else if ((dist + BulletGlobals.SIMD_EPSILON) >= penetrationDepth) {
					pointIndices[this.pointCount] = k;
					this.pointCount++;
				}
			}
		}

		for (int k = 0; k < this.pointCount; k++) {
			this.points[k].set(points[pointIndices[k]]);
		}
		
		intArrays.release(pointIndices);
	}

}
