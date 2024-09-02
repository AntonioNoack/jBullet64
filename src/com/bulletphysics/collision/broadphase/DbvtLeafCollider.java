package com.bulletphysics.collision.broadphase;

/**
 * @author jezek2
 * Dbvt implementation by Nathanael Presson
 */
public class DbvtLeafCollider extends Dbvt.ICollide {

    public DbvtBroadphase pbp;
    public DbvtProxy ppx;

    public DbvtLeafCollider(DbvtBroadphase p, DbvtProxy px) {
        this.pbp = p;
        this.ppx = px;
    }

    @Override
    public void process(Dbvt.Node na) {
        Dbvt.Node nb = ppx.leaf;
        if (nb != na) {
            DbvtProxy pa = (DbvtProxy) na.data;
            DbvtProxy pb = (DbvtProxy) nb.data;

            if (DbvtAabbMm.Intersect(pa.aabb, pb.aabb)) {
                if (pa.hashCode() > pb.hashCode()) {
                    DbvtProxy tmp = pa;
                    pa = pb;
                    pb = tmp;
                }
                pbp.paircache.addOverlappingPair(pa, pb);
            }
        }
    }

}
