package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.StridingMeshInterface;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * This class manages a sub part of a mesh supplied by the StridingMeshInterface interface.<p>
 * <p>
 * - Simply create this shape by passing the StridingMeshInterface to the constructor
 * GImpactMeshShapePart, then you must call updateBound() after creating the mesh<br>
 * - When making operations with this shape, you must call <b>lock</b> before accessing
 * to the trimesh primitives, and then call <b>unlock</b><br>
 * - You can handle deformable meshes with this shape, by calling postUpdate() every time
 * when changing the mesh vertices.
 *
 * @author jezek2
 */
public class GImpactMeshShapePart extends GImpactShapeInterface {

    TrimeshPrimitiveManager primitiveManager = new TrimeshPrimitiveManager();

    private final IntArrayList collided = new IntArrayList();

    public GImpactMeshShapePart() {
        boxSet.setPrimitiveManager(primitiveManager);
    }

    public GImpactMeshShapePart(StridingMeshInterface meshInterface, int part) {
        primitiveManager.meshInterface = meshInterface;
        primitiveManager.part = part;
        boxSet.setPrimitiveManager(primitiveManager);
    }

    @Override
    public boolean childrenHasTransform() {
        return false;
    }

    @Override
    public void lockChildShapes() {
        TrimeshPrimitiveManager dummyManager = (TrimeshPrimitiveManager) boxSet.getPrimitiveManager();
        dummyManager.lock();
    }

    @Override
    public void unlockChildShapes() {
        TrimeshPrimitiveManager dummymanager = (TrimeshPrimitiveManager) boxSet.getPrimitiveManager();
        dummymanager.unlock();
    }

    @Override
    public int getNumChildShapes() {
        return primitiveManager.getPrimitiveCount();
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
    PrimitiveManagerBase getPrimitiveManager() {
        return primitiveManager;
    }

    TrimeshPrimitiveManager getTrimeshPrimitiveManager() {
        return primitiveManager;
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        lockChildShapes();

        //#define CALC_EXACT_INERTIA 1
        //#ifdef CALC_EXACT_INERTIA
        inertia.set(0.0, 0.0, 0.0);

        int i = getVertexCount();
        double pointmass = mass / (double) i;

        Vector3d pointInertia = Stack.newVec();
        while ((i--) != 0) {
            getVertex(i, pointInertia);
            getPointInertia(pointInertia, pointmass, pointInertia);
            inertia.add(pointInertia);
        }
        Stack.subVec(1);

        unlockChildShapes();
    }

    @SuppressWarnings("UnusedReturnValue")
    private static Vector3d getPointInertia(Vector3d point, double mass, Vector3d out) {
        double x2 = point.x * point.x;
        double y2 = point.y * point.y;
        double z2 = point.z * point.z;
        out.set(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
        return out;
    }

    @Override
    ShapeType getGImpactShapeType() {
        return ShapeType.TRIMESH_SHAPE_PART;
    }

    @Override
    public boolean needsRetrieveTriangles() {
        return true;
    }

    @Override
    public boolean needsRetrieveTetrahedrons() {
        return false;
    }

    @Override
    public void getBulletTriangle(int prim_index, TriangleShapeEx triangle) {
        primitiveManager.getBulletTriangle(prim_index, triangle);
    }

    @Override
    void getBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron) {
        assert (false);
    }

    public int getVertexCount() {
        return primitiveManager.getVertexCount();
    }

    public void getVertex(int vertex_index, Vector3d vertex) {
        primitiveManager.getVertex(vertex_index, vertex);
    }

    @Override
    public void setMargin(double margin) {
        primitiveManager.margin = margin;
        postUpdate();
    }

    @Override
    public double getMargin() {
        return primitiveManager.margin;
    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        primitiveManager.scale.set(scaling);
        postUpdate();
    }

    @Override
    public Vector3d getLocalScaling(Vector3d out) {
        out.set(primitiveManager.scale);
        return out;
    }

    public int getPart() {
        return primitiveManager.part;
    }

    @Override
    public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
        lockChildShapes();
        AABB box = new AABB();
        box.min.set(aabbMin);
        box.max.set(aabbMax);

        collided.clear();
        boxSet.boxQuery(box, collided);

        if (collided.size() == 0) {
            unlockChildShapes();
            return;
        }

        int part = getPart();
        PrimitiveTriangle triangle = new PrimitiveTriangle();
        int i = collided.size();
        while ((i--) != 0) {
            getPrimitiveTriangle(collided.get(i), triangle);
            callback.processTriangle(triangle.vertices, part, collided.get(i));
        }
        unlockChildShapes();
    }

}
