package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector4d;


/**
 * PersistentManifold is a contact point cache, it stays persistent as long as objects
 * are overlapping in the broadphase. Those contact points are created by the collision
 * narrow phase.<p>
 * <p>
 * The cache can be empty, or hold 1, 2, 3 or 4 points. Some collision algorithms (GJK)
 * might only add one point at a time, updates/refreshes old contact points, and throw
 * them away if necessary (distance becomes too large).<p>
 * <p>
 * Reduces the cache to 4 points, when more then 4 points are added, using following rules:
 * the contact point with deepest penetration is always kept, and it tries to maximize the
 * area covered by the points.<p>
 * <p>
 * Note that some pairs of objects might have more then one contact manifold.
 *
 * @author jezek2
 */
public class PersistentManifold {

    //protected final BulletStack stack = BulletStack.get();

    public static final int MANIFOLD_CACHE_SIZE = 4;

    private final ManifoldPoint[] pointCache = new ManifoldPoint[MANIFOLD_CACHE_SIZE];
    // this two body pointers can point to the physics rigidbody class.
    // void* will allow any rigidbody class
    private Object body0;
    private Object body1;
    private int cachedPoints;

    public int index1a;

    {
        for (int i = 0; i < pointCache.length; i++) pointCache[i] = new ManifoldPoint();
    }

    public PersistentManifold() {
    }

    public PersistentManifold(Object body0, Object body1, int bla) {
        init(body0, body1, bla);
    }

    public void init(Object body0, Object body1, int bla) {
        this.body0 = body0;
        this.body1 = body1;
        cachedPoints = 0;
        index1a = 0;
    }

    // sort cached points so most isolated points come first
    private int sortCachedPoints(ManifoldPoint pt) {
        //calculate 4 possible cases areas, and take biggest area
        //also need to keep 'deepest'

        int maxPenetrationIndex = -1;
//#define KEEP_DEEPEST_POINT 1
//#ifdef KEEP_DEEPEST_POINT
        double maxPenetration = pt.getDistance();
        for (int i = 0; i < 4; i++) {
            if (pointCache[i].getDistance() < maxPenetration) {
                maxPenetrationIndex = i;
                maxPenetration = pointCache[i].getDistance();
            }
        }
//#endif //KEEP_DEEPEST_POINT

        double res0 = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0;
        if (maxPenetrationIndex != 0) {
            Vector3d a0 = Stack.newVec(pt.localPointA);
            a0.sub(pointCache[1].localPointA);

            Vector3d b0 = Stack.newVec(pointCache[3].localPointA);
            b0.sub(pointCache[2].localPointA);

            Vector3d cross = Stack.newVec();
            cross.cross(a0, b0);

            res0 = cross.lengthSquared();
        }

        if (maxPenetrationIndex != 1) {
            Vector3d a1 = Stack.newVec(pt.localPointA);
            a1.sub(pointCache[0].localPointA);

            Vector3d b1 = Stack.newVec(pointCache[3].localPointA);
            b1.sub(pointCache[2].localPointA);

            Vector3d cross = Stack.newVec();
            cross.cross(a1, b1);
            res1 = cross.lengthSquared();
        }

        if (maxPenetrationIndex != 2) {
            Vector3d a2 = Stack.newVec(pt.localPointA);
            a2.sub(pointCache[0].localPointA);

            Vector3d b2 = Stack.newVec(pointCache[3].localPointA);
            b2.sub(pointCache[1].localPointA);

            Vector3d cross = Stack.newVec();
            cross.cross(a2, b2);

            res2 = cross.lengthSquared();
        }

        if (maxPenetrationIndex != 3) {
            Vector3d a3 = Stack.newVec(pt.localPointA);
            a3.sub(pointCache[0].localPointA);

            Vector3d b3 = Stack.newVec(pointCache[2].localPointA);
            b3.sub(pointCache[1].localPointA);

            Vector3d cross = Stack.newVec();
            cross.cross(a3, b3);
            res3 = cross.lengthSquared();
        }

        Vector4d maxVec = new Vector4d();
        maxVec.set(res0, res1, res2, res3);
        return VectorUtil.closestAxis4(maxVec);
    }

    //private int findContactPoint(ManifoldPoint unUsed, int numUnused, ManifoldPoint pt);

    public Object getBody0() {
        return body0;
    }

    public Object getBody1() {
        return body1;
    }

    public void setBodies(Object body0, Object body1) {
        this.body0 = body0;
        this.body1 = body1;
    }

    public void clearUserCache(ManifoldPoint pt) {
        Object userPersistentData = pt.userPersistentData;
        if (userPersistentData != null) {
            if (BulletGlobals.getContactDestroyedCallback() != null) {
                BulletGlobals.getContactDestroyedCallback().contactDestroyed(userPersistentData);
                pt.userPersistentData = null;
            }
        }
    }

    public int getNumContacts() {
        return cachedPoints;
    }

    public ManifoldPoint getContactPoint(int index) {
        return pointCache[index];
    }

    // todo: get this margin from the current physics / collision environment
    public double getContactBreakingThreshold() {
        return BulletGlobals.getContactBreakingThreshold();
    }

    public int getCacheEntry(ManifoldPoint newPoint) {
        double shortestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
        int size = getNumContacts();
        int nearestPoint = -1;
        Vector3d diffA = Stack.newVec();
        for (int i = 0; i < size; i++) {
            ManifoldPoint mp = pointCache[i];

            diffA.sub(mp.localPointA, newPoint.localPointA);

            double distToManiPoint = diffA.dot(diffA);
            if (distToManiPoint < shortestDist) {
                shortestDist = distToManiPoint;
                nearestPoint = i;
            }
        }
        return nearestPoint;
    }

    public int addManifoldPoint(ManifoldPoint newPoint) {
        assert (validContactDistance(newPoint));

        int insertIndex = getNumContacts();
        if (insertIndex == MANIFOLD_CACHE_SIZE) {
            if (MANIFOLD_CACHE_SIZE >= 4) {
                //sort cache so best points come first, based on area
                insertIndex = sortCachedPoints(newPoint);
            } else {
                //#else
                insertIndex = 0;
            }
            clearUserCache(pointCache[insertIndex]);
        } else {
            cachedPoints++;
        }
        assert (pointCache[insertIndex].userPersistentData == null);
        pointCache[insertIndex].set(newPoint);
        return insertIndex;
    }

    public void removeContactPoint(int index) {
        clearUserCache(pointCache[index]);

        int lastUsedIndex = getNumContacts() - 1;
        if (index != lastUsedIndex) {
            // TODO: possible bug
            pointCache[index].set(pointCache[lastUsedIndex]);
            //get rid of duplicated userPersistentData pointer
            pointCache[lastUsedIndex].userPersistentData = null;
            pointCache[lastUsedIndex].appliedImpulse = 0.0;
            pointCache[lastUsedIndex].lateralFrictionInitialized = false;
            pointCache[lastUsedIndex].appliedImpulseLateral1 = 0.0;
            pointCache[lastUsedIndex].appliedImpulseLateral2 = 0.0;
            pointCache[lastUsedIndex].lifeTime = 0;
        }

        assert (pointCache[lastUsedIndex].userPersistentData == null);
        cachedPoints--;
    }

    public void replaceContactPoint(ManifoldPoint newPoint, int insertIndex) {
        assert (validContactDistance(newPoint));

        int lifeTime = pointCache[insertIndex].getLifeTime();
        double appliedImpulse = pointCache[insertIndex].appliedImpulse;
        double appliedLateralImpulse1 = pointCache[insertIndex].appliedImpulseLateral1;
        double appliedLateralImpulse2 = pointCache[insertIndex].appliedImpulseLateral2;

        assert (lifeTime >= 0);
        Object cache = pointCache[insertIndex].userPersistentData;

        pointCache[insertIndex].set(newPoint);

        pointCache[insertIndex].userPersistentData = cache;
        pointCache[insertIndex].appliedImpulse = appliedImpulse;
        pointCache[insertIndex].appliedImpulseLateral1 = appliedLateralImpulse1;
        pointCache[insertIndex].appliedImpulseLateral2 = appliedLateralImpulse2;

        pointCache[insertIndex].lifeTime = lifeTime;
    }

    private boolean validContactDistance(ManifoldPoint pt) {
        return pt.distance1 <= getContactBreakingThreshold();
    }

    // calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
    public void refreshContactPoints(Transform trA, Transform trB) {
        Vector3d tmp = Stack.newVec();
        int i;
//#ifdef DEBUG_PERSISTENCY
//	printf("refreshContactPoints posA = (%f,%f,%f) posB = (%f,%f,%f)\n",
//		trA.getOrigin().getX(),
//		trA.getOrigin().getY(),
//		trA.getOrigin().getZ(),
//		trB.getOrigin().getX(),
//		trB.getOrigin().getY(),
//		trB.getOrigin().getZ());
//#endif //DEBUG_PERSISTENCY
        // first refresh worldspace positions and distance
        for (i = getNumContacts() - 1; i >= 0; i--) {
            ManifoldPoint manifoldPoint = pointCache[i];

            manifoldPoint.positionWorldOnA.set(manifoldPoint.localPointA);
            trA.transform(manifoldPoint.positionWorldOnA);

            manifoldPoint.positionWorldOnB.set(manifoldPoint.localPointB);
            trB.transform(manifoldPoint.positionWorldOnB);

            tmp.set(manifoldPoint.positionWorldOnA);
            tmp.sub(manifoldPoint.positionWorldOnB);
            manifoldPoint.distance1 = tmp.dot(manifoldPoint.normalWorldOnB);

            manifoldPoint.lifeTime++;
        }

        // then
        double distance2d;
        Vector3d projectedDifference = Stack.newVec(), projectedPoint = Stack.newVec();

        for (i = getNumContacts() - 1; i >= 0; i--) {

            ManifoldPoint manifoldPoint = pointCache[i];
            // contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
            if (!validContactDistance(manifoldPoint)) {
                removeContactPoint(i);
            } else {
                // contact also becomes invalid when relative movement orthogonal to normal exceeds margin
                tmp.scale(manifoldPoint.distance1, manifoldPoint.normalWorldOnB);
                projectedPoint.sub(manifoldPoint.positionWorldOnA, tmp);
                projectedDifference.sub(manifoldPoint.positionWorldOnB, projectedPoint);
                distance2d = projectedDifference.dot(projectedDifference);
                if (distance2d > getContactBreakingThreshold() * getContactBreakingThreshold()) {
                    removeContactPoint(i);
                } else {
                    // contact point processed callback
                    if (BulletGlobals.getContactProcessedCallback() != null) {
                        BulletGlobals.getContactProcessedCallback().contactProcessed(manifoldPoint, body0, body1);
                    }
                }
            }
        }
    }

    public void clearManifold() {
        for (int i = 0; i < cachedPoints; i++) {
            clearUserCache(pointCache[i]);
        }
        cachedPoints = 0;
    }

}
