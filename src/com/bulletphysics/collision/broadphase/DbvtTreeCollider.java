// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

/**
 * @author jezek2
 */
public class DbvtTreeCollider extends Dbvt.ICollide {

    public DbvtBroadphase pbp;

    public DbvtTreeCollider(DbvtBroadphase p) {
        this.pbp = p;
    }

    @Override
    public void process(Dbvt.Node na, Dbvt.Node nb) {
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
