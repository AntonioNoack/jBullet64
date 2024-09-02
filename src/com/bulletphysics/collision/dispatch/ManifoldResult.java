package com.bulletphysics.collision.dispatch;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.narrowphase.DiscreteCollisionDetectorInterface;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ManifoldResult is helper class to manage contact results.
 *
 * @author jezek2
 */
public class ManifoldResult extends DiscreteCollisionDetectorInterface.Result {

    //protected final BulletStack stack = BulletStack.get();
    protected final ObjectPool<ManifoldPoint> pointsPool = ObjectPool.get(ManifoldPoint.class);

    private PersistentManifold manifoldPtr;

    // we need this for compounds
    private final Transform rootTransA = new Transform();
    private final Transform rootTransB = new Transform();
    private CollisionObject body0;
    private CollisionObject body1;
    private int partId0;
    private int partId1;
    private int index0;
    private int index1;

    public ManifoldResult() {
    }

    public ManifoldResult(CollisionObject body0, CollisionObject body1) {
        init(body0, body1);
    }

    public void init(CollisionObject body0, CollisionObject body1) {
        this.body0 = body0;
        this.body1 = body1;
        body0.getWorldTransform(this.rootTransA);
        body1.getWorldTransform(this.rootTransB);
    }

    public PersistentManifold getPersistentManifold() {
        return manifoldPtr;
    }

    public void setPersistentManifold(PersistentManifold manifoldPtr) {
        this.manifoldPtr = manifoldPtr;
    }

    public void setShapeIdentifiers(int partId0, int index0, int partId1, int index1) {
        this.partId0 = partId0;
        this.partId1 = partId1;
        this.index0 = index0;
        this.index1 = index1;
    }

    public void addContactPoint(Vector3d normalOnBInWorld, Vector3d pointInWorld, double depth) {
        assert (manifoldPtr != null);
        //order in manifold needs to match

        if (depth > manifoldPtr.getContactBreakingThreshold()) {
            return;
        }

        boolean isSwapped = manifoldPtr.getBody0() != body0;

        Vector3d pointA = Stack.newVec();
        pointA.scaleAdd(depth, normalOnBInWorld, pointInWorld);

        Vector3d localA = Stack.newVec();
        Vector3d localB = Stack.newVec();

        if (isSwapped) {
            rootTransB.invXform(pointA, localA);
            rootTransA.invXform(pointInWorld, localB);
        } else {
            rootTransA.invXform(pointA, localA);
            rootTransB.invXform(pointInWorld, localB);
        }

        ManifoldPoint newPt = pointsPool.get();
        newPt.init(localA, localB, normalOnBInWorld, depth);

        newPt.positionWorldOnA.set(pointA);
        newPt.positionWorldOnB.set(pointInWorld);

        int insertIndex = manifoldPtr.getCacheEntry(newPt);

        newPt.combinedFriction = calculateCombinedFriction(body0, body1);
        newPt.combinedRestitution = calculateCombinedRestitution(body0, body1);

        // BP mod, store contact triangles.
        newPt.partId0 = partId0;
        newPt.partId1 = partId1;
        newPt.index0 = index0;
        newPt.index1 = index1;

        // todo, check this for any side effects
        if (insertIndex >= 0) {
            //const btManifoldPoint& oldPoint = m_manifoldPtr->getContactPoint(insertIndex);
            manifoldPtr.replaceContactPoint(newPt, insertIndex);
        } else {
            insertIndex = manifoldPtr.addManifoldPoint(newPt);
        }

        // User can override friction and/or restitution
        if (BulletGlobals.getContactAddedCallback() != null &&
                // and if either of the two bodies requires custom material
                ((body0.getCollisionFlags() & CollisionFlags.CUSTOM_MATERIAL_CALLBACK) != 0 ||
                        (body1.getCollisionFlags() & CollisionFlags.CUSTOM_MATERIAL_CALLBACK) != 0)) {
            //experimental feature info, for per-triangle material etc.
            CollisionObject obj0 = isSwapped ? body1 : body0;
            CollisionObject obj1 = isSwapped ? body0 : body1;
            BulletGlobals.getContactAddedCallback().contactAdded(manifoldPtr.getContactPoint(insertIndex), obj0, partId0, index0, obj1, partId1, index1);
        }

        pointsPool.release(newPt);
    }

    // User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
    private static double calculateCombinedFriction(CollisionObject body0, CollisionObject body1) {
        double friction = body0.getFriction() * body1.getFriction();

        double MAX_FRICTION = 10f;
        if (friction < -MAX_FRICTION) {
            friction = -MAX_FRICTION;
        }
        if (friction > MAX_FRICTION) {
            friction = MAX_FRICTION;
        }
        return friction;
    }

    private static double calculateCombinedRestitution(CollisionObject body0, CollisionObject body1) {
        return body0.getRestitution() * body1.getRestitution();
    }

    public void refreshContactPoints() {
        assert (manifoldPtr != null);
        if (manifoldPtr.getNumContacts() == 0) {
            return;
        }

        boolean isSwapped = manifoldPtr.getBody0() != body0;

        if (isSwapped) {
            manifoldPtr.refreshContactPoints(rootTransB, rootTransA);
        } else {
            manifoldPtr.refreshContactPoints(rootTransA, rootTransB);
        }
    }
}
