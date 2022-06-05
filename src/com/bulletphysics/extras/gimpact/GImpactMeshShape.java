/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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
package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.dispatch.CollisionWorld.RayResultCallback;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.StridingMeshInterface;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.extras.gimpact.BoxCollision.AABB;
import com.bulletphysics.linearmath.Transform;
import java.util.ArrayList;
import cz.advel.stack.Stack;
import kotlin.NotImplementedError;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class GImpactMeshShape extends GImpactShapeInterface {

    protected ArrayList<GImpactMeshShapePart> meshParts = new ArrayList<>();

    public GImpactMeshShape(StridingMeshInterface meshInterface) {
        buildMeshParts(meshInterface);
    }

    public int getMeshPartCount() {
        return meshParts.size();
    }

    public GImpactMeshShapePart getMeshPart(int index) {
        return meshParts.get(index);
    }

    @Override
    public void setLocalScaling(Vector3d scaling) {
        localScaling.set(scaling);

        int i = meshParts.size();
        while ((i--) != 0) {
            GImpactMeshShapePart part = meshParts.get(i);
            part.setLocalScaling(scaling);
        }

        needsUpdate = true;
    }

    @Override
    public void setMargin(double margin) {
        collisionMargin = margin;

        int i = meshParts.size();
        while ((i--) != 0) {
            GImpactMeshShapePart part = meshParts.get(i);
            part.setMargin(margin);
        }

        needsUpdate = true;
    }

    @Override
    public void postUpdate() {
        int i = meshParts.size();
        while ((i--) != 0) {
            GImpactMeshShapePart part = meshParts.get(i);
            part.postUpdate();
        }

        needsUpdate = true;
    }

    @Override
    public void calculateLocalInertia(double mass, Vector3d inertia) {
        //#ifdef CALC_EXACT_INERTIA
        inertia.set(0.0, 0.0, 0.0);

        int i = getMeshPartCount();
        double partMass = mass / (double) i;

        Vector3d partInertia = Stack.newVec();
        while ((i--) != 0) {
            getMeshPart(i).calculateLocalInertia(partMass, partInertia);
            inertia.add(partInertia);
        }

        ////#else
        //
        //// Calc box inertia
        //
        //btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
        //btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
        //btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
        //const btScalar x2 = lx*lx;
        //const btScalar y2 = ly*ly;
        //const btScalar z2 = lz*lz;
        //const btScalar scaledmass = mass * btScalar(0.08333333);
        //
        //inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));
        ////#endif
    }

    @Override
    PrimitiveManagerBase getPrimitiveManager() {
        assert (false);
        return null;
    }

    @Override
    public int getNumChildShapes() {
        throw new NotImplementedError();
    }

    @Override
    public boolean childrenHasTransform() {
        throw new NotImplementedError();
    }

    @Override
    public boolean needsRetrieveTriangles() {
        throw new NotImplementedError();
    }

    @Override
    public boolean needsRetrieveTetrahedrons() {
        throw new NotImplementedError();
    }

    @Override
    public void getBulletTriangle(int prim_index, TriangleShapeEx triangle) {
        throw new NotImplementedError();
    }

    @Override
    void getBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron) {
        throw new NotImplementedError();
    }

    @Override
    public void lockChildShapes() {
        throw new NotImplementedError();
    }

    @Override
    public void unlockChildShapes() {
        throw new NotImplementedError();
    }

    @Override
    public void getChildAabb(int child_index, Transform t, Vector3d aabbMin, Vector3d aabbMax) {
        throw new NotImplementedError();
    }

    @Override
    public CollisionShape getChildShape(int index) {
        throw new NotImplementedError();
    }

    @Override
    public Transform getChildTransform(int index) {
        throw new NotImplementedError();
    }

    @Override
    public void setChildTransform(int index, Transform transform) {
        throw new NotImplementedError();
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
        int i = meshParts.size();
        while ((i--) != 0) {
            meshParts.get(i).processAllTriangles(callback, aabbMin, aabbMax);
        }
    }

    protected void buildMeshParts(StridingMeshInterface meshInterface) {
        for (int i = 0; i < meshInterface.getNumSubParts(); i++) {
            meshParts.add(new GImpactMeshShapePart(meshInterface, i));
        }
    }

    @Override
    protected void calcLocalAABB() {
        AABB tmpAABB = new AABB();

        localAABB.invalidate();
        int i = meshParts.size();
        while ((i--) != 0) {
            meshParts.get(i).updateBound();
            localAABB.merge(meshParts.get(i).getLocalBox(tmpAABB));
        }
    }

}
