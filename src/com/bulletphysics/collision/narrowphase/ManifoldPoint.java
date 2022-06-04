package com.bulletphysics.collision.narrowphase;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ManifoldPoint collects and maintains persistent contactpoints. Used to improve
 * stability and performance of rigidbody dynamics response.
 * 
 * @author jezek2
 */
public class ManifoldPoint {

	public final Vector3d localPointA = new Vector3d();
	public final Vector3d localPointB = new Vector3d();
	public final Vector3d positionWorldOnB = new Vector3d();
	// m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
	public final Vector3d positionWorldOnA = new Vector3d();
	public final Vector3d normalWorldOnB = new Vector3d();
	
	public double distance1;
	public double combinedFriction;
	public double combinedRestitution;
	
	// BP mod, store contact triangles.
	public int partId0;
	public int partId1;
	public int index0;
	public int index1;
	
	public Object userPersistentData;
	public double appliedImpulse;
	
	public boolean lateralFrictionInitialized;
	public double appliedImpulseLateral1;
	public double appliedImpulseLateral2;
	public int lifeTime; // lifetime of the contact point in frames

	public final Vector3d lateralFrictionDir1 = new Vector3d();
	public final Vector3d lateralFrictionDir2 = new Vector3d();
	
	public ManifoldPoint() {
		this.userPersistentData = null;
		this.appliedImpulse = 0.0;
		this.lateralFrictionInitialized = false;
		this.lifeTime = 0;
	}
	
	public ManifoldPoint(Vector3d pointA, Vector3d pointB, Vector3d normal, double distance) {
		init(pointA, pointB, normal, distance);
	}

	public void init(Vector3d pointA, Vector3d pointB, Vector3d normal, double distance) {
		this.localPointA.set(pointA);
		this.localPointB.set(pointB);
		this.normalWorldOnB.set(normal);
		this.distance1 = distance;
		this.combinedFriction = 0.0;
		this.combinedRestitution = 0.0;
		this.userPersistentData = null;
		this.appliedImpulse = 0.0;
		this.lateralFrictionInitialized = false;
		this.appliedImpulseLateral1 = 0.0;
		this.appliedImpulseLateral2 = 0.0;
		this.lifeTime = 0;
	}

	public double getDistance() {
		return distance1;
	}

	public int getLifeTime() {
		return lifeTime;
	}
	
	public void set(ManifoldPoint p) {
		localPointA.set(p.localPointA);
		localPointB.set(p.localPointB);
		positionWorldOnA.set(p.positionWorldOnA);
		positionWorldOnB.set(p.positionWorldOnB);
		normalWorldOnB.set(p.normalWorldOnB);
		distance1 = p.distance1;
		combinedFriction = p.combinedFriction;
		combinedRestitution = p.combinedRestitution;
		partId0 = p.partId0;
		partId1 = p.partId1;
		index0 = p.index0;
		index1 = p.index1;
		userPersistentData = p.userPersistentData;
		appliedImpulse = p.appliedImpulse;
		lateralFrictionInitialized = p.lateralFrictionInitialized;
		appliedImpulseLateral1 = p.appliedImpulseLateral1;
		appliedImpulseLateral2 = p.appliedImpulseLateral2;
		lifeTime = p.lifeTime;
		lateralFrictionDir1.set(p.lateralFrictionDir1);
		lateralFrictionDir2.set(p.lateralFrictionDir2);
	}
	
	public Vector3d getPositionWorldOnA(Vector3d out) {
		out.set(positionWorldOnA);
		return out;
		//return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
	}

	public Vector3d getPositionWorldOnB(Vector3d out) {
		out.set(positionWorldOnB);
		return out;
	}

	public void setDistance(double dist) {
		distance1 = dist;
	}
	
}
