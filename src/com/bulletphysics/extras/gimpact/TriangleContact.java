package com.bulletphysics.extras.gimpact;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.util.ArrayPool;
import java.util.ArrayList;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;

/**
 *
 * @author jezek2
 */
public class TriangleContact {
	
	private final ArrayPool<int[]> intArrays = ArrayPool.get(int.class);
	
	public static final int MAX_TRI_CLIPPING = 16;

    public double penetration_depth;
    public int pointCount;
    public final Vector4d separatingNormal = new Vector4d();
    public Vector3d[] points = new Vector3d[MAX_TRI_CLIPPING];

	public TriangleContact() {
		for (int i=0; i<points.length; i++) {
			points[i] = new Vector3d();
		}
	}

	public TriangleContact(TriangleContact other) {
		copyFrom(other);
	}

	public void set(TriangleContact other) {
		copyFrom(other);
	}
	
	public void copyFrom(TriangleContact other) {
		penetration_depth = other.penetration_depth;
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
	public void mergePoints(Vector4d plane, double margin, ArrayList<Vector3d> points, int point_count) {

		this.pointCount = 0;
		penetration_depth = -1000.0;

		int[] point_indices = intArrays.getFixed(MAX_TRI_CLIPPING);

		for (int _k = 0; _k < point_count; _k++) {
			double _dist = -ClipPolygon.distancePointPlane(plane, points.get(_k)) + margin;

			if (_dist >= 0.0f) {
				if (_dist > penetration_depth) {
					penetration_depth = _dist;
					point_indices[0] = _k;
					this.pointCount = 1;
				}
				else if ((_dist + BulletGlobals.SIMD_EPSILON) >= penetration_depth) {
					point_indices[this.pointCount] = _k;
					this.pointCount++;
				}
			}
		}

		for (int _k = 0; _k < this.pointCount; _k++) {
			this.points[_k].set(points.get(point_indices[_k]));
		}
		
		intArrays.release(point_indices);
	}

}
