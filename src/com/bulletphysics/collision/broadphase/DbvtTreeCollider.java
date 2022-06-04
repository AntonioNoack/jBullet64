
// Dbvt implementation by Nathanael Presson
package com.bulletphysics.collision.broadphase;

/**
 *
 * @author jezek2
 */
public class DbvtTreeCollider extends Dbvt.ICollide {

	public DbvtBroadphase pbp;

	public DbvtTreeCollider(DbvtBroadphase p) {
		this.pbp = p;
	}

	@Override
	public void Process(Dbvt.Node na, Dbvt.Node nb) {
		DbvtProxy pa = (DbvtProxy) na.data;
		DbvtProxy pb = (DbvtProxy) nb.data;
		//#if DBVT_BP_DISCRETPAIRS
		if (DbvtAabbMm.Intersect(pa.aabb, pb.aabb))
		//#endif
		{
			//if(pa>pb) btSwap(pa,pb);
			if (pa.hashCode() > pb.hashCode()) {
				DbvtProxy tmp = pa;
				pa = pb;
				pb = tmp;
			}
			pbp.paircache.addOverlappingPair(pa, pb);
		}
	}

}
