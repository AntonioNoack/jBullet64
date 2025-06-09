// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class DbvtBroadphase extends BroadphaseInterface {

    public static final double DBVT_BP_MARGIN = 0.05f;

    public static final int DYNAMIC_SET = 0; // Dynamic set index
    public static final int FIXED_SET = 1; // Fixed set index
    public static final int STAGE_COUNT = 2; // Number of stages

    public final Dbvt[] sets = new Dbvt[2];                        // Dbvt sets
    public DbvtProxy[] stageRoots = new DbvtProxy[STAGE_COUNT + 1]; // Stages list
    public OverlappingPairCache paircache;                         // Pair cache
    public double predictedframes;                                  // Frames predicted
    public int stageCurrent;                                       // Current stage
    public int fupdates;                                           // % of fixed updates per frame
    public int dupdates;                                           // % of dynamic updates per frame
    public int pid;                                                // Parse id
    public int gid;                                                // Gen id
    public boolean releasepaircache;                               // Release pair cache on delete

    public DbvtBroadphase(OverlappingPairCache paircache) {
        sets[0] = new Dbvt();
        sets[1] = new Dbvt();

        //Dbvt.benchmark();
        releasepaircache = (paircache == null);
        predictedframes = 2;
        stageCurrent = 0;
        fupdates = 1;
        dupdates = 1;
        this.paircache = (paircache != null ? paircache : new HashedOverlappingPairCache());
        gid = 0;
        pid = 0;

        for (int i = 0; i <= STAGE_COUNT; i++) {
            stageRoots[i] = null;
        }
        //#if DBVT_BP_PROFILE
        //clear(m_profiling);
        //#endif
    }

    public void collide(Dispatcher dispatcher) {
        //SPC(m_profiling.m_total);

        // optimize:
        sets[0].optimizeIncremental(1 + (sets[0].leaves * dupdates) / 100);
        sets[1].optimizeIncremental(1 + (sets[1].leaves * fupdates) / 100);

        // dynamic -> fixed set:
        stageCurrent = (stageCurrent + 1) % STAGE_COUNT;
        DbvtProxy current = stageRoots[stageCurrent];
        if (current != null) {
            DbvtTreeCollider collider = new DbvtTreeCollider(this);
            do {
                DbvtProxy next = current.links[1];
                stageRoots[current.stage] = listRemove(current, stageRoots[current.stage]);
                stageRoots[STAGE_COUNT] = listAppend(current, stageRoots[STAGE_COUNT]);
                Dbvt.collideTT(sets[1].root, current.leaf, collider);
                sets[0].remove(current.leaf);
                current.leaf = sets[1].insert(current.aabb, current);
                current.stage = STAGE_COUNT;
                current = next;
            } while (current != null);
        }

        // collide dynamics:
        {
            DbvtTreeCollider collider = new DbvtTreeCollider(this);
            Dbvt.collideTT(sets[0].root, sets[1].root, collider);
            Dbvt.collideTT(sets[0].root, sets[0].root, collider);
        }

        collideCleanup(dispatcher);
        pid++;
    }

    private void collideCleanup(Dispatcher dispatcher) {
        ObjectArrayList<BroadphasePair> pairs = paircache.getOverlappingPairArray();
        if (!pairs.isEmpty()) {
            for (int i = 0, ni = pairs.size(); i < ni; i++) {
                BroadphasePair p = pairs.getQuick(i);
                DbvtProxy pa = (DbvtProxy) p.proxy0;
                DbvtProxy pb = (DbvtProxy) p.proxy1;
                if (!DbvtAabbMm.Intersect(pa.aabb, pb.aabb)) {
                    //if(pa>pb) btSwap(pa,pb);
                    if (pa.hashCode() > pb.hashCode()) {
                        DbvtProxy tmp = pa;
                        pa = pb;
                        pb = tmp;
                    }
                    paircache.removeOverlappingPair(pa, pb, dispatcher);
                    ni--;
                    i--;
                }
            }
        }
    }

    private static DbvtProxy listAppend(DbvtProxy item, DbvtProxy list) {
        item.links[0] = null;
        item.links[1] = list;
        if (list != null) list.links[0] = item;
        list = item;
        return list;
    }

    private static DbvtProxy listRemove(DbvtProxy item, DbvtProxy list) {
        if (item.links[0] != null) {
            item.links[0].links[1] = item.links[1];
        } else {
            list = item.links[1];
        }

        if (item.links[1] != null) {
            item.links[1].links[0] = item.links[0];
        }
        return list;
    }

    public BroadphaseProxy createProxy(
            Vector3d aabbMin, Vector3d aabbMax, BroadphaseNativeType shapeType, Object userPtr,
            short collisionFilterGroup, short collisionFilterMask, Dispatcher dispatcher, Object multiSapProxy) {
        DbvtProxy proxy = new DbvtProxy(userPtr, collisionFilterGroup, collisionFilterMask);
        DbvtAabbMm.FromMM(aabbMin, aabbMax, proxy.aabb);
        proxy.leaf = sets[0].insert(proxy.aabb, proxy);
        proxy.stage = stageCurrent;
        proxy.uniqueId = ++gid;
        stageRoots[stageCurrent] = listAppend(proxy, stageRoots[stageCurrent]);
        return (proxy);
    }

    public void destroyProxy(BroadphaseProxy absproxy, Dispatcher dispatcher) {
        DbvtProxy proxy = (DbvtProxy) absproxy;
        if (proxy.stage == STAGE_COUNT) {
            sets[1].remove(proxy.leaf);
        } else {
            sets[0].remove(proxy.leaf);
        }
        stageRoots[proxy.stage] = listRemove(proxy, stageRoots[proxy.stage]);
        paircache.removeOverlappingPairsContainingProxy(proxy, dispatcher);
        //btAlignedFree(proxy);
    }

    public void setAabb(BroadphaseProxy absproxy, Vector3d aabbMin, Vector3d aabbMax, Dispatcher dispatcher) {
        DbvtProxy proxy = (DbvtProxy) absproxy;
        DbvtAabbMm aabb = DbvtAabbMm.FromMM(aabbMin, aabbMax, Stack.newDbvtAabbMm());

        if (proxy.stage == STAGE_COUNT) {
            // fixed -> dynamic set
            sets[1].remove(proxy.leaf);
            proxy.leaf = sets[0].insert(aabb, proxy);
        } else {
            // dynamic set:
            if (DbvtAabbMm.Intersect(proxy.leaf.volume, aabb)) {/* Moving				*/
                Vector3d delta = Stack.newVec();
                delta.add(aabbMin, aabbMax);
                delta.scale(0.5);
                delta.sub(proxy.aabb.Center(Stack.newVec()));
                //#ifdef DBVT_BP_MARGIN
                delta.scale(predictedframes);
                sets[0].update(proxy.leaf, aabb, delta, DBVT_BP_MARGIN);
                Stack.subVec(2);
                //#else
                //m_sets[0].update(proxy->leaf,aabb,delta*m_predictedframes);
                //#endif
            } else {
                // teleporting:
                sets[0].update(proxy.leaf, aabb);
            }
        }

        stageRoots[proxy.stage] = listRemove(proxy, stageRoots[proxy.stage]);
        proxy.aabb.set(aabb);
        proxy.stage = stageCurrent;
        stageRoots[stageCurrent] = listAppend(proxy, stageRoots[stageCurrent]);
        Stack.subDbvtAabbMm(1);
    }

    public void calculateOverlappingPairs(Dispatcher dispatcher) {
        collide(dispatcher);
    }

    public OverlappingPairCache getOverlappingPairCache() {
        return paircache;
    }

    public void getBroadphaseAabb(Vector3d aabbMin, Vector3d aabbMax) {
        DbvtAabbMm bounds = Stack.newDbvtAabbMm();
        if (!sets[0].isEmpty()) {
            if (!sets[1].isEmpty()) {
                DbvtAabbMm.Merge(sets[0].root.volume, sets[1].root.volume, bounds);
            } else {
                bounds.set(sets[0].root.volume);
            }
        } else if (!sets[1].isEmpty()) {
            bounds.set(sets[1].root.volume);
        } else {
            DbvtAabbMm.FromCR(Stack.newVec(), 0.0, bounds);
            Stack.subVec(1);
        }
        aabbMin.set(bounds.Mins());
        aabbMax.set(bounds.Maxs());
        Stack.subDbvtAabbMm(1);
    }
}
