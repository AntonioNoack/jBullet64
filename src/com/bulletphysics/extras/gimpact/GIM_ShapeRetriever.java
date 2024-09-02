package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.CollisionShape;

/**
 *
 * @author jezek2
 */
class GIM_ShapeRetriever {

	public GImpactShapeInterface gim_shape;
	public TriangleShapeEx trishape = new TriangleShapeEx();
	public TetrahedronShapeEx tetrashape = new TetrahedronShapeEx();

	public ChildShapeRetriever child_retriever = new ChildShapeRetriever();
	public TriangleShapeRetriever tri_retriever = new TriangleShapeRetriever();
	public TetraShapeRetriever tetra_retriever = new TetraShapeRetriever();
	public ChildShapeRetriever current_retriever;

	public GIM_ShapeRetriever(GImpactShapeInterface gim_shape) {
		this.gim_shape = gim_shape;
		
		// select retriever
		if (gim_shape.needsRetrieveTriangles()) {
			current_retriever = tri_retriever;
		}
		else if (gim_shape.needsRetrieveTetrahedrons()) {
			current_retriever = tetra_retriever;
		}
		else {
			current_retriever = child_retriever;
		}

		current_retriever.parent = this;
	}

	public CollisionShape getChildShape(int index) {
		return current_retriever.getChildShape(index);
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	public static class ChildShapeRetriever {
		public GIM_ShapeRetriever parent;

		public CollisionShape getChildShape(int index) {
			return parent.gim_shape.getChildShape(index);
		}
	}

	public static class TriangleShapeRetriever extends ChildShapeRetriever {
		@Override
		public CollisionShape getChildShape(int index) {
			parent.gim_shape.getBulletTriangle(index, parent.trishape);
			return parent.trishape;
		}
	}

	public static class TetraShapeRetriever extends ChildShapeRetriever {
		@Override
		public CollisionShape getChildShape(int index) {
			parent.gim_shape.getBulletTetrahedron(index, parent.tetrashape);
			return parent.tetrashape;
		}
	}
	
}
