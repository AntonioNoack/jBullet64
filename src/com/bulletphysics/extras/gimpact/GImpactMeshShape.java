package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.dispatch.CollisionWorld.RayResultCallback;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.StridingMeshInterface;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;
import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
public class GImpactMeshShape extends GImpactShapeInterface {
	
	protected ObjectArrayList<GImpactMeshShapePart> mesh_parts = new ObjectArrayList<GImpactMeshShapePart>();

	public GImpactMeshShape(StridingMeshInterface meshInterface) {
		buildMeshParts(meshInterface);
	}
	
	public int getMeshPartCount() {
		return mesh_parts.size();
	}

	public GImpactMeshShapePart getMeshPart(int index) {
		return mesh_parts.getQuick(index);
	}

	@Override
	public void setLocalScaling(Vector3d scaling) {
		localScaling.set(scaling);

		int i = mesh_parts.size();
		while ((i--) != 0) {
			GImpactMeshShapePart part = mesh_parts.getQuick(i);
			part.setLocalScaling(scaling);
		}

		needs_update = true;
	}

	@Override
	public void setMargin(double margin) {
		collisionMargin = margin;

		int i = mesh_parts.size();
		while ((i--) != 0) {
			GImpactMeshShapePart part = mesh_parts.getQuick(i);
			part.setMargin(margin);
		}

		needs_update = true;
	}

	@Override
	public void postUpdate() {
		int i = mesh_parts.size();
		while ((i--) != 0) {
			GImpactMeshShapePart part = mesh_parts.getQuick(i);
			part.postUpdate();
		}

		needs_update = true;
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		inertia.set(0.0, 0.0, 0.0);

		int i = getMeshPartCount();
		double partialMass = mass / (double) i;

		Vector3d partialInertia = Stack.newVec();
		while ((i--) != 0) {
			getMeshPart(i).calculateLocalInertia(partialMass, partialInertia);
			inertia.add(partialInertia);
		}
		Stack.subVec(1);
	}
	
	@Override
	PrimitiveManagerBase getPrimitiveManager() {
		assert (false);
		return null;
	}

	@Override
	public int getNumChildShapes() {
		assert (false);
		return 0;
	}

	@Override
	public boolean childrenHasTransform() {
		assert (false);
		return false;
	}

	@Override
	public boolean needsRetrieveTriangles() {
		assert (false);
		return false;
	}

	@Override
	public boolean needsRetrieveTetrahedrons() {
		assert (false);
		return false;
	}

	@Override
	public void getBulletTriangle(int prim_index, TriangleShapeEx triangle) {
		assert (false);
	}

	@Override
	void getBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron) {
		assert (false);
	}

	@Override
	public void lockChildShapes() {
		assert (false);
	}

	@Override
	public void unlockChildShapes() {
		assert (false);
	}

	@Override
	public void getChildAabb(int child_index, Transform t, Vector3d aabbMin, Vector3d aabbMax) {
		assert (false);
	}

	@Override
	public CollisionShape getChildShape(int index) {
		assert (false);
		return null;
	}

	@Override
	public Transform getChildTransform(int index) {
		assert (false);
		return null;
	}

	@Override
	public void setChildTransform(int index, Transform transform) {
		assert (false);
	}

	@Override
	ShapeType getGImpactShapeType() {
		return ShapeType.TRIMESH_SHAPE;
	}

	@Override
	public String getName() {
		return "GImpactMesh";
	}

	@Override
	public void rayTest(Vector3d rayFrom, Vector3d rayTo, RayResultCallback resultCallback) {
	}

	@Override
	public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
		int i = mesh_parts.size();
		while ((i--) != 0) {
			mesh_parts.getQuick(i).processAllTriangles(callback, aabbMin, aabbMax);
		}
	}
	
	protected void buildMeshParts(StridingMeshInterface meshInterface) {
		for (int i=0; i<meshInterface.getNumSubParts(); i++) {
			GImpactMeshShapePart newpart = new GImpactMeshShapePart(meshInterface, i);
			mesh_parts.add(newpart);
		}
	}

	@Override
	protected void calcLocalAABB() {
		AABB tmpAABB = new AABB();

		localAABB.invalidate();
		int i = mesh_parts.size();
		while ((i--) != 0) {
			mesh_parts.getQuick(i).updateBound();
			localAABB.merge(mesh_parts.getQuick(i).getLocalBox(tmpAABB));
		}
	}

}
