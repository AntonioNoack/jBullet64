package com.bulletphysics.dynamics.character;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.dynamics.ActionInterface;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * KinematicCharacterController is an object that supports a sliding motion in
 * a world. It uses a {@link GhostObject} and convex sweep test to test for upcoming
 * collisions. This is combined with discrete collision detection to recover
 * from penetrations.<p>
 * <p>
 * Interaction between KinematicCharacterController and dynamic rigid bodies
 * needs to be explicitly implemented by the user.
 *
 * @author tomrbryn
 */
public class KinematicCharacterController extends ActionInterface {

    private static final Vector3d[] upAxisDirection = new Vector3d[]{
            new Vector3d(1.0, 0.0, 0.0),
            new Vector3d(0.0, 1.0, 0.0),
            new Vector3d(0.0, 0.0, 1.0),
    };

    private final PairCachingGhostObject ghostObject;

    // is also in ghostObject, but it needs to be convex, so we store it here
    // to avoid upcast
    private final ConvexShape convexShape;

    protected double verticalVelocity;
    protected double verticalOffset;

    protected double fallSpeed;
    protected double jumpSpeed;
    protected double maxJumpHeight;

    protected double maxSlopeRadians; // Slope angle that is set (used for returning the exact value)
    protected double maxSlopeCosine; // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)

    protected double gravity;

    protected double turnAngle;

    protected double stepHeight;

    protected double addedMargin; // @todo: remove this and fix the code

    // this is the desired walk direction, set by the user
    private final Vector3d walkDirection = new Vector3d();
    protected Vector3d normalizedDirection = new Vector3d();

    // some internal variables
    protected Vector3d currentPosition = new Vector3d();
    protected double currentStepOffset;
    protected Vector3d targetPosition = new Vector3d();

    // keep track of the contact manifolds
    ObjectArrayList<PersistentManifold> manifoldArray = new ObjectArrayList<PersistentManifold>();

    protected boolean touchingContact;
    protected Vector3d touchingNormal = new Vector3d();

    protected boolean wasOnGround;

    public boolean useGhostObjectSweepTest;
    private boolean useWalkDirection;
    private double velocityTimeInterval;
    public int upAxis;

    @SuppressWarnings("unused")
    public KinematicCharacterController(PairCachingGhostObject ghostObject, ConvexShape convexShape, double stepHeight) {
        this(ghostObject, convexShape, stepHeight, 1);
    }

    public KinematicCharacterController(PairCachingGhostObject ghostObject, ConvexShape convexShape, double stepHeight, int upAxis) {
        this.upAxis = upAxis;
        this.addedMargin = 0.02f;
        this.walkDirection.set(0, 0, 0);
        this.useGhostObjectSweepTest = true;
        this.ghostObject = ghostObject;
        this.stepHeight = stepHeight;
        this.turnAngle = 0.0;
        this.convexShape = convexShape;
        this.useWalkDirection = true;
        this.velocityTimeInterval = 0.0;
        this.verticalVelocity = 0.0;
        this.verticalOffset = 0.0;
        this.gravity = 9.8f; // 1G acceleration
        this.fallSpeed = 55.0; // Terminal velocity of a sky diver in m/s.
        this.jumpSpeed = 10.0; // ?
        this.wasOnGround = false;
        setMaxSlope((50.0 / 180.0) * Math.PI);
    }

    private PairCachingGhostObject getGhostObject() {
        return ghostObject;
    }

    // ActionInterface interface
    public void updateAction(CollisionWorld collisionWorld, double deltaTime) {
        preStep(collisionWorld);
        playerStep(collisionWorld, deltaTime);
    }

    // ActionInterface interface
    public void debugDraw(IDebugDraw debugDrawer) {
    }

    /**
     * This should probably be called setPositionIncrementPerSimulatorStep. This
     * is neither a direction nor a velocity, but the amount to increment the
     * position each simulation iteration, regardless of dt.<p>
     * <p>
     * This call will reset any velocity set by {@link #setVelocityForTimeInterval}.
     */
    @SuppressWarnings("unused")
    public void setWalkDirection(Vector3d walkDirection) {
        useWalkDirection = true;
        this.walkDirection.set(walkDirection);
        normalizedDirection.set(getNormalizedVector(walkDirection, Stack.newVec()));
    }

    /**
     * Caller provides a velocity with which the character should move for the
     * given time period. After the time period, velocity is reset to zero.
     * This call will reset any walk direction set by {@link #setWalkDirection}.
     * Negative time intervals will result in no motion.
     */
    @SuppressWarnings("unused")
    public void setVelocityForTimeInterval(Vector3d velocity, double timeInterval) {
        useWalkDirection = false;
        walkDirection.set(velocity);
        normalizedDirection.set(getNormalizedVector(walkDirection, Stack.newVec()));
        velocityTimeInterval = timeInterval;
    }

    public void reset() {
    }

    public void warp(Vector3d origin) {
        Transform xform = Stack.newTrans();
        xform.setIdentity();
        xform.origin.set(origin);
        ghostObject.setWorldTransform(xform);
    }

    public void preStep(CollisionWorld collisionWorld) {
        int numPenetrationLoops = 0;
        touchingContact = false;
        while (recoverFromPenetration(collisionWorld)) {
            numPenetrationLoops++;
            touchingContact = true;
            if (numPenetrationLoops > 4) {
                //printf("character could not recover from penetration = %d\n", numPenetrationLoops);
                break;
            }
        }

        currentPosition.set(ghostObject.getWorldTransform(Stack.newTrans()).origin);
        targetPosition.set(currentPosition);
        //printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);
    }

    public void playerStep(CollisionWorld collisionWorld, double dt) {
        //printf("playerStep(): ");
        //printf("  dt = %f", dt);

        // quick check...
        if (!useWalkDirection && velocityTimeInterval <= 0.0) {
            //printf("\n");
            return; // no motion
        }

        wasOnGround = onGround();

        // Update fall velocity.
        verticalVelocity -= gravity * dt;
        if (verticalVelocity > 0.0 && verticalVelocity > jumpSpeed) {
            verticalVelocity = jumpSpeed;
        }
        if (verticalVelocity < 0.0 && Math.abs(verticalVelocity) > Math.abs(fallSpeed)) {
            verticalVelocity = -Math.abs(fallSpeed);
        }
        verticalOffset = verticalVelocity * dt;

        Transform xform = ghostObject.getWorldTransform(Stack.newTrans());

        //printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
        //printf("walkSpeed=%f\n",walkSpeed);

        stepUp(collisionWorld);
        if (useWalkDirection) {
            //System.out.println("playerStep 3");
            stepForwardAndStrafe(collisionWorld, walkDirection);
        } else {
            System.out.println("playerStep 4");
            //printf("  time: %f", m_velocityTimeInterval);

            // still have some time left for moving!
            double dtMoving = Math.min(dt, velocityTimeInterval);
            velocityTimeInterval -= dt;

            // how far will we move while we are moving?
            Vector3d move = Stack.newVec();
            move.scale(dtMoving, walkDirection);

            //printf("  dtMoving: %f", dtMoving);

            // okay, step
            stepForwardAndStrafe(collisionWorld, move);
        }
        stepDown(collisionWorld, dt);

        //printf("\n");

        xform.origin.set(currentPosition);
        ghostObject.setWorldTransform(xform);
    }

    @SuppressWarnings("unused")
    public void setFallSpeed(double fallSpeed) {
        this.fallSpeed = fallSpeed;
    }

    @SuppressWarnings("unused")
    public void setJumpSpeed(double jumpSpeed) {
        this.jumpSpeed = jumpSpeed;
    }

    @SuppressWarnings("unused")
    public void setMaxJumpHeight(double maxJumpHeight) {
        this.maxJumpHeight = maxJumpHeight;
    }

    public boolean canJump() {
        return onGround();
    }

    @SuppressWarnings("unused")
    public void jump() {
        if (!canJump()) return;

        verticalVelocity = jumpSpeed;

        //#if 0
        //currently no jumping.
        //btTransform xform;
        //m_rigidBody->getMotionState()->getWorldTransform (xform);
        //btVector3 up = xform.getBasis()[1];
        //up.normalize ();
        //btScalar magnitude = (btScalar(1.0)/m_rigidBody->getInvMass()) * btScalar(8.0);
        //m_rigidBody->applyCentralImpulse (up * magnitude);
        //#endif
    }

    public void setGravity(double gravity) {
        this.gravity = gravity;
    }

    public double getGravity() {
        return gravity;
    }

    public void setMaxSlope(double slopeRadians) {
        maxSlopeRadians = slopeRadians;
        maxSlopeCosine = (double) Math.cos((double) slopeRadians);
    }

    public double getMaxSlope() {
        return maxSlopeRadians;
    }

    public boolean onGround() {
        return verticalVelocity == 0.0 && verticalOffset == 0.0;
    }

    // static helper method
    private static Vector3d getNormalizedVector(Vector3d v, Vector3d out) {
        out.set(v);
        out.normalize();
        if (out.length() < BulletGlobals.SIMD_EPSILON) {
            out.set(0, 0, 0);
        }
        return out;
    }

    /**
     * Returns the reflection direction of a ray going 'direction' hitting a surface
     * with normal 'normal'.<p>
     * <p>
     * From: <a href="http://www-cs-students.stanford.edu/~adityagp/final/node3.html">Stanford-Article</a>
     */
    protected Vector3d computeReflectionDirection(Vector3d direction, Vector3d normal, Vector3d out) {
        // return direction - (btScalar(2.0) * direction.dot(normal)) * normal;
        out.set(normal);
        out.scale(-2.0 * direction.dot(normal));
        out.add(direction);
        return out;
    }

    /**
     * Returns the portion of 'direction' that is parallel to 'normal'
     */
    protected Vector3d parallelComponent(Vector3d direction, Vector3d normal, Vector3d out) {
        //btScalar magnitude = direction.dot(normal);
        //return normal * magnitude;
        out.set(normal);
        out.scale(direction.dot(normal));
        return out;
    }

    /**
     * Returns the portion of 'direction' that is perpindicular to 'normal'
     */
    protected Vector3d perpindicularComponent(Vector3d direction, Vector3d normal, Vector3d out) {
        //return direction - parallelComponent(direction, normal);
        Vector3d perpendicular = parallelComponent(direction, normal, out);
        perpendicular.scale(-1);
        perpendicular.add(direction);
        return perpendicular;
    }

    protected boolean recoverFromPenetration(CollisionWorld collisionWorld) {
        boolean penetration = false;

        collisionWorld.getDispatcher().dispatchAllCollisionPairs(
                ghostObject.getOverlappingPairCache(), collisionWorld.getDispatchInfo(), collisionWorld.getDispatcher());

        currentPosition.set(ghostObject.getWorldTransform(Stack.newTrans()).origin);

        double maxPen = 0.0;
        for (int i = 0; i < ghostObject.getOverlappingPairCache().getNumOverlappingPairs(); i++) {
            manifoldArray.clear();

            BroadphasePair collisionPair = ghostObject.getOverlappingPairCache().getOverlappingPairArray().getQuick(i);

            if (collisionPair.algorithm != null) {
                collisionPair.algorithm.getAllContactManifolds(manifoldArray);
            }

            for (int j = 0; j < manifoldArray.size(); j++) {
                PersistentManifold manifold = manifoldArray.getQuick(j);
                double directionSign = manifold.getBody0() == ghostObject ? -1.0 : 1.0;
                for (int p = 0; p < manifold.getNumContacts(); p++) {
                    ManifoldPoint pt = manifold.getContactPoint(p);

                    double dist = pt.getDistance();
                    if (dist < 0.0) {
                        if (dist < maxPen) {
                            maxPen = dist;
                            touchingNormal.set(pt.normalWorldOnB);//??
                            touchingNormal.scale(directionSign);
                        }

                        currentPosition.scaleAdd(directionSign * dist * 0.2, pt.normalWorldOnB, currentPosition);

                        penetration = true;
                    } // else printf("touching %f\n", dist);
                }

                //manifold->clearManifold();
            }
        }

        Transform newTrans = ghostObject.getWorldTransform(Stack.newTrans());
        newTrans.origin.set(currentPosition);
        ghostObject.setWorldTransform(newTrans);
        //printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);

        //System.out.println("recoverFromPenetration "+penetration+" "+touchingNormal);

        return penetration;
    }

    protected void stepUp(CollisionWorld world) {
        // phase 1: up
        Transform start = Stack.newTrans();
        Transform end = Stack.newTrans();
        targetPosition.scaleAdd(stepHeight + Math.max(verticalOffset, 0.0), upAxisDirection[upAxis], currentPosition);

        start.setIdentity();
        end.setIdentity();

        /* FIXME: Handle penetration properly */
        start.origin.scaleAdd(convexShape.getMargin() + addedMargin, upAxisDirection[upAxis], currentPosition);
        end.origin.set(targetPosition);

        // Find only sloped/flat surface hits, avoid wall and ceiling hits...
        Vector3d up = Stack.newVec();
        up.scale(-1.0, upAxisDirection[upAxis]);
        KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(ghostObject, up, 0.0);
        callback.collisionFilterGroup = getGhostObject().getBroadphaseHandle().collisionFilterGroup;
        callback.collisionFilterMask = getGhostObject().getBroadphaseHandle().collisionFilterMask;

        if (useGhostObjectSweepTest) {
            ghostObject.convexSweepTest(convexShape, start, end, callback, world.getDispatchInfo().allowedCcdPenetration);
        } else {
            world.convexSweepTest(convexShape, start, end, callback);
        }

        if (callback.hasHit()) {
            // we moved up only a fraction of the step height
            currentStepOffset = stepHeight * callback.closestHitFraction;
            currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
            verticalVelocity = 0.0;
            verticalOffset = 0.0;
        } else {
            currentStepOffset = stepHeight;
            currentPosition.set(targetPosition);
        }
    }

    protected void updateTargetPositionBasedOnCollision(Vector3d hitNormal) {
        updateTargetPositionBasedOnCollision(hitNormal, 0.0, 1.0);
    }

    protected void updateTargetPositionBasedOnCollision(Vector3d hitNormal, double tangentMag, double normalMag) {
        Vector3d movementDirection = Stack.newVec();
        movementDirection.sub(targetPosition, currentPosition);
        double movementLength = movementDirection.length();
        if (movementLength > BulletGlobals.SIMD_EPSILON) {
            movementDirection.normalize();

            Vector3d reflectDir = computeReflectionDirection(movementDirection, hitNormal, Stack.newVec());
            reflectDir.normalize();

            Vector3d parallelDir = parallelComponent(reflectDir, hitNormal, Stack.newVec());
            Vector3d perpindicularDir = perpindicularComponent(reflectDir, hitNormal, Stack.newVec());

            targetPosition.set(currentPosition);
            if (false) //tangentMag != 0.0)
            {
                Vector3d parComponent = Stack.newVec();
                parComponent.scale(tangentMag * movementLength, parallelDir);
                //printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
                targetPosition.add(parComponent);
            }

            if (normalMag != 0.0) {
                Vector3d perpComponent = Stack.newVec();
                perpComponent.scale(normalMag * movementLength, perpindicularDir);
                //printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
                targetPosition.add(perpComponent);
            }
        } // else printf("movementLength don't normalize a zero vector\n");
    }

    protected void stepForwardAndStrafe(CollisionWorld collisionWorld, Vector3d walkMove) {
        // printf("m_normalizedDirection=%f,%f,%f\n",
        // 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
        // phase 2: forward and strafe
        Transform start = Stack.newTrans();
        Transform end = Stack.newTrans();
        targetPosition.add(currentPosition, walkMove);
        start.setIdentity();
        end.setIdentity();

        double fraction = 1.0;
        Vector3d distance2Vec = Stack.newVec();
        distance2Vec.sub(currentPosition, targetPosition);
        double distance2 = distance2Vec.lengthSquared();
        //printf("distance2=%f\n",distance2);

		/*if (touchingContact) {
			if (normalizedDirection.dot(touchingNormal) > 0.0) {
				updateTargetPositionBasedOnCollision(touchingNormal);
			}
		}*/

        Vector3d hitDistanceVec = Stack.newVec();
        Vector3d currentDir = Stack.newVec();

        int maxIter = 10;
        while (fraction > 0.01f && maxIter-- > 0) {
            start.origin.set(currentPosition);
            end.origin.set(targetPosition);

            KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(ghostObject, upAxisDirection[upAxis], -1.0);
            callback.collisionFilterGroup = getGhostObject().getBroadphaseHandle().collisionFilterGroup;
            callback.collisionFilterMask = getGhostObject().getBroadphaseHandle().collisionFilterMask;

            double margin = convexShape.getMargin();
            convexShape.setMargin(margin + addedMargin);

            if (useGhostObjectSweepTest) {
                ghostObject.convexSweepTest(convexShape, start, end, callback, collisionWorld.getDispatchInfo().allowedCcdPenetration);
            } else {
                collisionWorld.convexSweepTest(convexShape, start, end, callback);
            }

            convexShape.setMargin(margin);

            fraction -= callback.closestHitFraction;

            if (callback.hasHit()) {
                // we moved only a fraction
                hitDistanceVec.sub(callback.hitPointWorld, currentPosition);
                //double hitDistance = hitDistanceVec.length();

                // if the distance is farther than the collision margin, move
                //if (hitDistance > addedMargin) {
                //	//printf("callback.m_closestHitFraction=%f\n",callback.m_closestHitFraction);
                //	currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
                //}

                updateTargetPositionBasedOnCollision(callback.hitNormalWorld);

                currentDir.sub(targetPosition, currentPosition);
                distance2 = currentDir.lengthSquared();
                if (distance2 > BulletGlobals.SIMD_EPSILON) {
                    currentDir.normalize();
                    // see Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners."
                    if (currentDir.dot(normalizedDirection) <= 0.0) {
                        break;
                    }
                } else {
                    //printf("currentDir: don't normalize a zero vector\n");
                    break;
                }
            } else {
                // we moved whole way
                currentPosition.set(targetPosition);
            }

            //if (callback.m_closestHitFraction == 0.f)
            //    break;
        }

        Stack.subTrans(2);
        Stack.subVec(3);
    }

    protected void stepDown(CollisionWorld collisionWorld, double dt) {
        Transform start = Stack.newTrans();
        Transform end = Stack.newTrans();

        // phase 3: down
        double additionalDownStep = (wasOnGround /*&& !onGround()*/) ? stepHeight : 0.0;
        Vector3d stepDrop = Stack.newVec();
        stepDrop.scale(currentStepOffset + additionalDownStep, upAxisDirection[upAxis]);
        double downVelocity = (additionalDownStep == 0.0 && verticalVelocity < 0.0 ? -verticalVelocity : 0.0) * dt;
        Vector3d gravityDrop = Stack.newVec();
        gravityDrop.scale(downVelocity, upAxisDirection[upAxis]);
        targetPosition.sub(stepDrop);
        targetPosition.sub(gravityDrop);

        start.setIdentity();
        end.setIdentity();

        start.origin.set(currentPosition);
        end.origin.set(targetPosition);

        KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(ghostObject, upAxisDirection[upAxis], maxSlopeCosine);
        callback.collisionFilterGroup = getGhostObject().getBroadphaseHandle().collisionFilterGroup;
        callback.collisionFilterMask = getGhostObject().getBroadphaseHandle().collisionFilterMask;

        if (useGhostObjectSweepTest) {
            ghostObject.convexSweepTest(convexShape, start, end, callback, collisionWorld.getDispatchInfo().allowedCcdPenetration);
        } else {
            collisionWorld.convexSweepTest(convexShape, start, end, callback);
        }

        if (callback.hasHit()) {
            // we dropped a fraction of the height -> hit floor
            currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
            verticalVelocity = 0.0;
            verticalOffset = 0.0;
        } else {
            // we dropped the full height
            currentPosition.set(targetPosition);
        }
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static class KinematicClosestNotMeRayResultCallback extends CollisionWorld.ClosestRayResultCallback {
        protected CollisionObject me;

        public KinematicClosestNotMeRayResultCallback(CollisionObject me) {
            super(new Vector3d(), new Vector3d());
            this.me = me;
        }

        @Override
        public double addSingleResult(CollisionWorld.LocalRayResult rayResult, boolean normalInWorldSpace) {
            if (rayResult.collisionObject == me) {
                return 1.0;
            }

            return super.addSingleResult(rayResult, normalInWorldSpace);
        }
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static class KinematicClosestNotMeConvexResultCallback extends CollisionWorld.ClosestConvexResultCallback {
        protected CollisionObject me;
        protected final Vector3d up;
        protected double minSlopeDot;

        public KinematicClosestNotMeConvexResultCallback(CollisionObject me, final Vector3d up, double minSlopeDot) {
            init(new Vector3d(), new Vector3d());
            this.me = me;
            this.up = up;
            this.minSlopeDot = minSlopeDot;
        }

        @Override
        public double addSingleResult(CollisionWorld.LocalConvexResult convexResult, boolean normalInWorldSpace) {
            if (convexResult.hitCollisionObject == me) {
                return 1.0;
            }

            Vector3d hitNormalWorld;
            if (normalInWorldSpace) {
                hitNormalWorld = convexResult.hitNormalLocal;
            } else {
                //need to transform normal into worldspace
                hitNormalWorld = Stack.newVec();
                hitCollisionObject.getWorldTransform(Stack.newTrans()).basis.transform(convexResult.hitNormalLocal, hitNormalWorld);
            }

            double dotUp = up.dot(hitNormalWorld);
            if (dotUp < minSlopeDot) {
                return 1.0;
            }

            return super.addSingleResult(convexResult, normalInWorldSpace);
        }
    }

}
