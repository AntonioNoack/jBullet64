
// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

import java.util.ArrayList;
import java.util.List;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class DbvtBroadphase extends BroadphaseInterface {

    public static final double DBVT_BP_MARGIN = 0.05f;

    public static final int DYNAMIC_SET = 0; // Dynamic set index
    public static final int FIXED_SET = 1; // Fixed set index
    public static final int STAGECOUNT = 2; // Number of stages

    public final Dbvt[] sets = new Dbvt[2];                        // Dbvt sets
    public DbvtProxy[] stageRoots = new DbvtProxy[STAGECOUNT + 1]; // Stages list
    public OverlappingPairCache paircache;                         // Pair cache
    public double predictedframes;                                  // Frames predicted
    public int stageCurrent;                                       // Current stage
    public int fupdates;                                           // % of fixed updates per frame
    public int dupdates;                                           // % of dynamic updates per frame
    public int pid;                                                // Parse id
    public int gid;                                                // Gen id
    public boolean releasepaircache;                               // Release pair cache on delete

    //#if DBVT_BP_PROFILE
    //btClock					m_clock;
    //struct	{
    //		unsigned long		m_total;
    //		unsigned long		m_ddcollide;
    //		unsigned long		m_fdcollide;
    //		unsigned long		m_cleanup;
    //		unsigned long		m_jobcount;
    //		}				m_profiling;
    //#endif

    public DbvtBroadphase() {
        this(null);
    }

    public DbvtBroadphase(OverlappingPairCache paircache) {
        sets[0] = new Dbvt();
        sets[1] = new Dbvt();

        //Dbvt.benchmark();
        releasepaircache = (paircache != null ? false : true);
        predictedframes = 2;
        stageCurrent = 0;
        fupdates = 1;
        dupdates = 1;
        this.paircache = (paircache != null ? paircache : new HashedOverlappingPairCache());
        gid = 0;
        pid = 0;

        for (int i = 0; i <= STAGECOUNT; i++) {
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
        stageCurrent = (stageCurrent + 1) % STAGECOUNT;
        DbvtProxy current = stageRoots[stageCurrent];
        if (current != null) {
            DbvtTreeCollider collider = new DbvtTreeCollider(this);
            do {
                DbvtProxy next = current.links[1];
                stageRoots[current.stage] = listremove(current, stageRoots[current.stage]);
                stageRoots[STAGECOUNT] = listappend(current, stageRoots[STAGECOUNT]);
                Dbvt.collideTT(sets[1].root, current.leaf, collider);
                sets[0].remove(current.leaf);
                current.leaf = sets[1].insert(current.aabb, current);
                current.stage = STAGECOUNT;
                current = next;
            } while (current != null);
        }

        // collide dynamics:
        {
            DbvtTreeCollider collider = new DbvtTreeCollider(this);
            {
                //SPC(m_profiling.m_fdcollide);
                Dbvt.collideTT(sets[0].root, sets[1].root, collider);
            }
            {
                //SPC(m_profiling.m_ddcollide);
                Dbvt.collideTT(sets[0].root, sets[0].root, collider);
            }
        }

        // clean up:
        {
            //SPC(m_profiling.m_cleanup);
            List<BroadphasePair> pairs = paircache.getOverlappingPairArray();
            if (pairs.size() > 0) {
                for (int i = 0, ni = pairs.size(); i < ni; i++) {
                    BroadphasePair p = pairs.get(i);
                    DbvtProxy pa = (DbvtProxy) p.pProxy0;
                    DbvtProxy pb = (DbvtProxy) p.pProxy1;
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
        pid++;
    }

    private static DbvtProxy listappend(DbvtProxy item, DbvtProxy list) {
        item.links[0] = null;
        item.links[1] = list;
        if (list != null) list.links[0] = item;
        list = item;
        return list;
    }

    private static DbvtProxy listremove(DbvtProxy item, DbvtProxy list) {
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

    public BroadphaseProxy createProxy(Vector3d aabbMin, Vector3d aabbMax, BroadphaseNativeType shapeType, Object userPtr, short collisionFilterGroup, short collisionFilterMask, Dispatcher dispatcher, Object multiSapProxy) {
        DbvtProxy proxy = new DbvtProxy(userPtr, collisionFilterGroup, collisionFilterMask);
        DbvtAabbMm.FromMM(aabbMin, aabbMax, proxy.aabb);
        proxy.leaf = sets[0].insert(proxy.aabb, proxy);
        proxy.stage = stageCurrent;
        proxy.uniqueId = ++gid;
        stageRoots[stageCurrent] = listappend(proxy, stageRoots[stageCurrent]);
        return (proxy);
    }

    public void destroyProxy(BroadphaseProxy absproxy, Dispatcher dispatcher) {
        DbvtProxy proxy = (DbvtProxy) absproxy;
        if (proxy.stage == STAGECOUNT) {
            sets[1].remove(proxy.leaf);
        } else {
            sets[0].remove(proxy.leaf);
        }
        stageRoots[proxy.stage] = listremove(proxy, stageRoots[proxy.stage]);
        paircache.removeOverlappingPairsContainingProxy(proxy, dispatcher);
        //btAlignedFree(proxy);
    }

    public void setAabb(BroadphaseProxy absproxy, Vector3d aabbMin, Vector3d aabbMax, Dispatcher dispatcher) {
        DbvtProxy proxy = (DbvtProxy) absproxy;
        DbvtAabbMm aabb = DbvtAabbMm.FromMM(aabbMin, aabbMax, Stack.borrowAABBMM);
        if (proxy.stage == STAGECOUNT) {
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

        stageRoots[proxy.stage] = listremove(proxy, stageRoots[proxy.stage]);
        proxy.aabb.set(aabb);
        proxy.stage = stageCurrent;
        stageRoots[stageCurrent] = listappend(proxy, stageRoots[stageCurrent]);
    }

    public void calculateOverlappingPairs(Dispatcher dispatcher) {
        collide(dispatcher);

        //#if DBVT_BP_PROFILE
        //if(0==(m_pid%DBVT_BP_PROFILING_RATE))
        //	{
        //	printf("fixed(%u) dynamics(%u) pairs(%u)\r\n",m_sets[1].m_leafs,m_sets[0].m_leafs,m_paircache->getNumOverlappingPairs());
        //	printf("mode:    %s\r\n",m_mode==MODE_FULL?"full":"incremental");
        //	printf("cleanup: %s\r\n",m_cleanupmode==CLEANUP_FULL?"full":"incremental");
        //	unsigned int	total=m_profiling.m_total;
        //	if(total<=0) total=1;
        //	printf("ddcollide: %u%% (%uus)\r\n",(50+m_profiling.m_ddcollide*100)/total,m_profiling.m_ddcollide/DBVT_BP_PROFILING_RATE);
        //	printf("fdcollide: %u%% (%uus)\r\n",(50+m_profiling.m_fdcollide*100)/total,m_profiling.m_fdcollide/DBVT_BP_PROFILING_RATE);
        //	printf("cleanup:   %u%% (%uus)\r\n",(50+m_profiling.m_cleanup*100)/total,m_profiling.m_cleanup/DBVT_BP_PROFILING_RATE);
        //	printf("total:     %uus\r\n",total/DBVT_BP_PROFILING_RATE);
        //	const unsigned long	sum=m_profiling.m_ddcollide+
        //							m_profiling.m_fdcollide+
        //							m_profiling.m_cleanup;
        //	printf("leaked: %u%% (%uus)\r\n",100-((50+sum*100)/total),(total-sum)/DBVT_BP_PROFILING_RATE);
        //	printf("job counts: %u%%\r\n",(m_profiling.m_jobcount*100)/((m_sets[0].m_leafs+m_sets[1].m_leafs)*DBVT_BP_PROFILING_RATE));
        //	clear(m_profiling);
        //	m_clock.reset();
        //	}
        //#endif
    }

    public OverlappingPairCache getOverlappingPairCache() {
        return paircache;
    }

    public void getBroadphaseAabb(Vector3d aabbMin, Vector3d aabbMax) {
        DbvtAabbMm bounds = new DbvtAabbMm();
        if (!sets[0].empty()) {
            if (!sets[1].empty()) {
                DbvtAabbMm.Merge(sets[0].root.volume, sets[1].root.volume, bounds);
            } else {
                bounds.set(sets[0].root.volume);
            }
        } else if (!sets[1].empty()) {
            bounds.set(sets[1].root.volume);
        } else {
            DbvtAabbMm.FromCR(Stack.newVec(0.0), 0.0, bounds);
        }
        aabbMin.set(bounds.Mins());
        aabbMax.set(bounds.Maxs());
    }

}
