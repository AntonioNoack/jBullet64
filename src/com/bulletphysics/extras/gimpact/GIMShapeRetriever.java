package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.CollisionShape;

/**
 * @author jezek2
 */
class GIMShapeRetriever {

    private final GImpactShapeInterface gimShape;

    private TriangleShapeEx triShape;
    private TetrahedronShapeEx tetraShape;

    public GIMShapeRetriever(GImpactShapeInterface gimShape) {
        this.gimShape = gimShape;

        // select retriever
        if (gimShape.needsRetrieveTriangles()) {
            triShape = new TriangleShapeEx();
        } else if (gimShape.needsRetrieveTetrahedrons()) {
            tetraShape = new TetrahedronShapeEx();
        }
    }

    public CollisionShape getChildShape(int index) {
        if (triShape != null) {
            gimShape.getBulletTriangle(index, triShape);
            return triShape;
        } else if (tetraShape != null) {
            gimShape.getBulletTetrahedron(index, tetraShape);
            return tetraShape;
        } else {
            return gimShape.getChildShape(index);
        }
    }
}
