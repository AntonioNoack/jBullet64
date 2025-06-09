package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.dispatch.CollisionWorld.RayResultCallback;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3d;

/**
 * Base class for gimpact shapes.
 * 
 * @author jezek2
 */
public abstract class GImpactShapeInterface extends ConcaveShape {

    protected AABB localAABB = new AABB();
    protected boolean needsUpdate;
    protected final Vector3d localScaling = new Vector3d();
    GImpactBvh box_set = new GImpactBvh(); // optionally boxset

	public GImpactShapeInterface() {
		localAABB.invalidate();
		needsUpdate = true;
		localScaling.set(1.0, 1.0, 1.0);
	}

	/**
	 * Performs refit operation.<p>
	 * Updates the entire Box set of this shape.<p>
	 * 
	 * postUpdate() must be called for attemps to calculating the box set, else this function
	 * will does nothing.<p>
	 * 
	 * if m_needs_update == true, then it calls calcLocalAABB();
	 */
	public void updateBound() {
		if (!needsUpdate) {
			return;
		}
		calcLocalAABB();
		needsUpdate = false;
	}

	/**
	 * If the Bounding box is not updated, then this class attemps to calculate it.<p>
     * Calls updateBound() for update the box set.
     */
	@Override
	public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		AABB transformedbox = new AABB(localAABB);
		transformedbox.applyTransform(t);
		aabbMin.set(transformedbox.min);
		aabbMax.set(transformedbox.max);
	}

	/**
	 * Tells to this object that is needed to refit the box set.
	 */
	public void postUpdate() {
		needsUpdate = true;
	}
	
	/**
	 * Obtains the local box, which is the global calculated box of the total of subshapes.
	 */
	public AABB getLocalBox(AABB out) {
		out.set(localAABB);
		return out;
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.GIMPACT_SHAPE_PROXYTYPE;
	}

	/**
	 * You must call updateBound() for update the box set.
	 */
	@Override
	public void setLocalScaling(Vector3d scaling) {
		localScaling.set(scaling);
		postUpdate();
	}

	@Override
	public Vector3d getLocalScaling(Vector3d out) {
		out.set(localScaling);
		return out;
	}

	@Override
	public void setMargin(double margin) {
		collisionMargin = margin;
		int i = getNumChildShapes();
		while ((i--) != 0) {
			CollisionShape child = getChildShape(i);
			child.setMargin(margin);
		}

		needsUpdate = true;
	}

	/**
	 * Base method for determining which kind of GIMPACT shape we get.
	 */
	abstract ShapeType getGImpactShapeType();
	
	GImpactBvh getBoxSet() {
		return box_set;
	}

	/**
	 * Determines if this class has a hierarchy structure for sorting its primitives.
	 */
	public boolean hasBoxSet() {
        return box_set.getNodeCount() != 0;
    }

	/**
	 * Obtains the primitive manager.
	 */
	abstract PrimitiveManagerBase getPrimitiveManager();

	/**
	 * Gets the number of children.
	 */
	public abstract int getNumChildShapes();

	/**
	 * If true, then its children must get transforms.
	 */
	public abstract boolean childrenHasTransform();

	/**
	 * Determines if this shape has triangles.
	 */
	public abstract boolean needsRetrieveTriangles();

	/**
	 * Determines if this shape has tetrahedrons.
	 */
	public abstract boolean needsRetrieveTetrahedrons();

	public abstract void getBulletTriangle(int prim_index, TriangleShapeEx triangle);

	abstract void getBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron);

	/**
	 * Call when reading child shapes.
	 */
	public void lockChildShapes() {
	}

	public void unlockChildShapes() {
	}
	
	/**
	 * If this trimesh.
	 */
	void getPrimitiveTriangle(int index, PrimitiveTriangle triangle) {
		getPrimitiveManager().getPrimitiveTriangle(index, triangle);
	}
	
	/**
	 * Use this function for perfofm refit in bounding boxes.
	 */
	protected void calcLocalAABB() {
		lockChildShapes();
		if (box_set.getNodeCount() == 0) {
			box_set.buildSet();
		}
		else {
			box_set.update();
		}
		unlockChildShapes();

		box_set.getGlobalBox(localAABB);
	}
	
	/**
	 * Retrieves the bound from a child.
	 */
	public void getChildAabb(int child_index, Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		AABB child_aabb = new AABB();
		getPrimitiveManager().getPrimitiveBox(child_index, child_aabb);
		child_aabb.applyTransform(t);
		aabbMin.set(child_aabb.min);
		aabbMax.set(child_aabb.max);
	}

	/**
	 * Gets the children.
	 */
	public abstract CollisionShape getChildShape(int index);
	
	/**
	 * Gets the children transform.
	 */
	public abstract Transform getChildTransform(int index);

	/**
	 * Sets the children transform.<p>
	 * You must call updateBound() for update the box set.
	 */
	public abstract void setChildTransform(int index, Transform transform);

	/**
	 * Virtual method for ray collision.
	 */
	public void rayTest(Vector3d rayFrom, Vector3d rayTo, RayResultCallback resultCallback) {
	}
	
	/**
	 * Function for retrieve triangles. It gives the triangles in local space.
	 */
	@Override
	public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
	}
	
}
