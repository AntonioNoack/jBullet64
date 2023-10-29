package com.bulletphysics.dynamics;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.broadphase.*;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.SimulationIslandManager;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.vehicle.RaycastVehicle;
import com.bulletphysics.dynamics.vehicle.WheelInfo;
import com.bulletphysics.linearmath.*;

import java.util.ArrayList;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;
import java.util.Comparator;

/**
 * DiscreteDynamicsWorld provides discrete rigid body simulation.
 *
 * @author jezek2
 */
public class DiscreteDynamicsWorld extends DynamicsWorld {

    protected ConstraintSolver constraintSolver;
    protected SimulationIslandManager islandManager;
    protected final ArrayList<TypedConstraint> constraints = new ArrayList<>();
    protected final Vector3d gravity = new Vector3d(0.0, -10.0, 0.0);

    // for variable time steps
    protected double localTime = 1.0 / 60.0;
    // for variable time steps

    protected boolean ownsIslandManager;
    protected boolean ownsConstraintSolver;

    public ArrayList<RaycastVehicle> vehicles = new ArrayList<>();

    public ArrayList<ActionInterface> actions = new ArrayList<>();

	public DiscreteDynamicsWorld(Dispatcher dispatcher, BroadphaseInterface pairCache, ConstraintSolver constraintSolver, CollisionConfiguration collisionConfiguration) {
        super(dispatcher, pairCache, collisionConfiguration);
        this.constraintSolver = constraintSolver;

        if (this.constraintSolver == null) {
            this.constraintSolver = new SequentialImpulseConstraintSolver();
            ownsConstraintSolver = true;
        } else {
            ownsConstraintSolver = false;
        }

		islandManager = new SimulationIslandManager();
        ownsIslandManager = true;
    }

    protected void saveKinematicState(double timeStep) {
        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null) {
                //Transform predictedTrans = new Transform();
                if (body.getActivationState() != CollisionObject.ISLAND_SLEEPING) {
                    if (body.isKinematicObject()) {
                        // to calculate velocities next frame
                        body.saveKinematicState(timeStep);
                    }
                }
            }
        }
    }

    @Override
    public void debugDrawWorld() {
        if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_CONTACT_POINTS) != 0) {
            int numManifolds = getDispatcher().getNumManifolds();
            Vector3d color = Stack.newVec();
            color.set(0.0, 0.0, 0.0);
            for (int i = 0; i < numManifolds; i++) {
                PersistentManifold contactManifold = getDispatcher().getManifoldByIndexInternal(i);
                int numContacts = contactManifold.getNumContacts();
                for (int j = 0; j < numContacts; j++) {
                    ManifoldPoint cp = contactManifold.getContactPoint(j);
                    getDebugDrawer().drawContactPoint(cp.positionWorldOnB, cp.normalWorldOnB, cp.getDistance(), cp.getLifeTime(), color);
                }
            }
            Stack.subVec(1);
        }

        if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & (DebugDrawModes.DRAW_WIREFRAME | DebugDrawModes.DRAW_AABB)) != 0) {
            int i;

            Transform tmpTrans = Stack.newTrans();
            Vector3d minAabb = Stack.newVec();
            Vector3d maxAabb = Stack.newVec();
            Vector3d colorvec = Stack.newVec();

            // todo: iterate over awake simulation islands!
            for (i = 0; i < collisionObjects.size(); i++) {
                CollisionObject colObj = collisionObjects.get(i);
                if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_WIREFRAME) != 0) {
                    Vector3d color = Stack.newVec();
                    color.set(255f, 255f, 255f);
                    switch (colObj.getActivationState()) {
                        case CollisionObject.ACTIVE_TAG:
                            color.set(255f, 255f, 255f);
                            break;
                        case CollisionObject.ISLAND_SLEEPING:
                            color.set(0f, 255f, 0.0);
                            break;
                        case CollisionObject.WANTS_DEACTIVATION:
                            color.set(0f, 255f, 255f);
                            break;
                        case CollisionObject.DISABLE_DEACTIVATION:
                            color.set(255f, 0.0, 0.0);
                            break;
                        case CollisionObject.DISABLE_SIMULATION:
                            color.set(255f, 255f, 0.0);
                            break;
                        default: {
                            color.set(255f, 0.0, 0.0);
                        }
                    }

                    debugDrawObject(colObj.getWorldTransform(tmpTrans), colObj.getCollisionShape(), color);
                }
                if (debugDrawer != null && (debugDrawer.getDebugMode() & DebugDrawModes.DRAW_AABB) != 0) {
                    colorvec.set(1.0, 0.0, 0.0);
                    colObj.getCollisionShape().getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
                    debugDrawer.drawAabb(minAabb, maxAabb, colorvec);
                }
            }

            Vector3d wheelColor = Stack.newVec();
            Vector3d wheelPosWS = Stack.newVec();
            Vector3d axle = Stack.newVec();
            Vector3d tmp = Stack.newVec();

            for (i = 0; i < vehicles.size(); i++) {
                RaycastVehicle vehicle = vehicles.get(i);
                for (int v = 0; v < vehicle.getNumWheels(); v++) {
                    wheelColor.set(0, 255, 255);
                    WheelInfo info = vehicle.getWheelInfo(v);
                    if (info.raycastInfo.isInContact) {
                        wheelColor.set(0, 0, 255);
                    } else {
                        wheelColor.set(255, 0, 255);
                    }

                    wheelPosWS.set(info.worldTransform.origin);

                    int rightAxis = vehicle.getRightAxis();
                    axle.set(
                            info.worldTransform.basis.getElement(0, rightAxis),
                            info.worldTransform.basis.getElement(1, rightAxis),
                            info.worldTransform.basis.getElement(2, rightAxis));


                    //m_vehicles[i]->getWheelInfo(v).m_raycastInfo.m_wheelAxleWS
                    //debug wheels (cylinders)
                    tmp.add(wheelPosWS, axle);
                    debugDrawer.drawLine(wheelPosWS, tmp, wheelColor);
                    debugDrawer.drawLine(wheelPosWS, info.raycastInfo.contactPointWS, wheelColor);
                }
            }

            if (getDebugDrawer() != null && getDebugDrawer().getDebugMode() != 0) {
                for (i = 0; i < actions.size(); i++) {
                    actions.get(i).debugDraw(debugDrawer);
                }
            }
        }
    }

    @Override
    public void clearForces() {
        // todo: iterate over awake simulation islands!
        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null) {
                body.clearForces();
            }
        }
    }

    /**
     * Apply gravity, call this once per timestep.
     */
    public void applyGravity() {
        // todo: iterate over awake simulation islands!
        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null && body.isActive()) {
                body.applyGravity();
            }
        }
    }

    protected void synchronizeMotionStates() {
        Transform interpolatedTransform = Stack.newTrans();

        Transform tmpTrans = Stack.newTrans();
        Vector3d tmpLinVel = Stack.newVec();
        Vector3d tmpAngVel = Stack.newVec();

        // todo: iterate over awake simulation islands!
        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null && body.getMotionState() != null && !body.isStaticOrKinematicObject()) {
                // we need to call the update at least once, even for sleeping objects
                // otherwise the 'graphics' transform never updates properly
                // so todo: add 'dirty' flag
                //if (body->getActivationState() != ISLAND_SLEEPING)
                {
                    TransformUtil.integrateTransform(
                            body.getInterpolationWorldTransform(tmpTrans),
                            body.getInterpolationLinearVelocity(tmpLinVel),
                            body.getInterpolationAngularVelocity(tmpAngVel),
                            localTime * body.getHitFraction(), interpolatedTransform);
                    body.getMotionState().setWorldTransform(interpolatedTransform);
                }
            }
        }

        Stack.subTrans(2);
        Stack.subVec(2);

        if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_WIREFRAME) != 0) {
            for (RaycastVehicle vehicle : vehicles) {
                for (int v = 0; v < vehicle.getNumWheels(); v++) {
                    // synchronize the wheels with the (interpolated) chassis worldtransform
                    vehicle.updateWheelTransform(v, true);
                }
            }
        }

    }

    @Override
    public int stepSimulation(double timeStep, int maxSubSteps, double fixedTimeStep) {
        startProfiling(timeStep);

        long t0 = System.nanoTime();

        try {
            int numSimulationSubSteps = 0;

            if (maxSubSteps != 0) {
                // fixed timestep with interpolation
                localTime += timeStep;
                if (localTime >= fixedTimeStep) {
                    numSimulationSubSteps = (int) (localTime / fixedTimeStep);
                    localTime -= numSimulationSubSteps * fixedTimeStep;
                }
            } else {
                //variable timestep
                fixedTimeStep = timeStep;
                localTime = timeStep;
                if (!ScalarUtil.fuzzyZero(timeStep)) {
                    numSimulationSubSteps = 1;
                    maxSubSteps = 1;
                }
            }

            // process some debugging flags
            if (getDebugDrawer() != null) {
                BulletGlobals.setDeactivationDisabled((getDebugDrawer().getDebugMode() & DebugDrawModes.NO_DEACTIVATION) != 0);
            }
            if (numSimulationSubSteps != 0) {
                saveKinematicState(fixedTimeStep);

                applyGravity();

                // clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
                int clampedSimulationSteps = Math.min(numSimulationSubSteps, maxSubSteps);

                for (int i = 0; i < clampedSimulationSteps; i++) {
                    internalSingleStepSimulation(fixedTimeStep);
                    synchronizeMotionStates();
                }
            }

            synchronizeMotionStates();

            clearForces();

            return numSimulationSubSteps;
        } finally {
            BulletStats.stepSimulationTime = (System.nanoTime() - t0) / 1000000;
        }
    }

    protected void internalSingleStepSimulation(double timeStep) {
        // apply gravity, predict motion
        predictUnconstraintMotion(timeStep);

        DispatcherInfo dispatchInfo = getDispatchInfo();

        dispatchInfo.timeStep = timeStep;
        dispatchInfo.stepCount = 0;
        dispatchInfo.debugDraw = getDebugDrawer();

        // perform collision detection
        performDiscreteCollisionDetection();

        calculateSimulationIslands();

        getSolverInfo().timeStep = timeStep;

        // solve contact and other joint constraints
        solveConstraints(getSolverInfo());

        //CallbackTriggers();

        // integrate transforms
        integrateTransforms(timeStep);

        // update vehicle simulation
        updateActions(timeStep);

        // update vehicle simulation
        updateVehicles(timeStep);

        updateActivationState(timeStep);

        if (internalTickCallback != null) {
            internalTickCallback.internalTick(this, timeStep);
        }
    }

    @Override
    public void setGravity(Vector3d gravity) {
        this.gravity.set(gravity);
        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null) {
                body.setGravity(gravity);
            }
        }
    }

    @Override
    public Vector3d getGravity(Vector3d out) {
        out.set(gravity);
        return out;
    }

    @Override
    public void removeRigidBody(RigidBody body) {
        removeCollisionObject(body);
    }

    @Override
    public void addRigidBody(RigidBody body) {
        if (!body.isStaticOrKinematicObject()) {
            body.setGravity(gravity);
        }

        if (body.getCollisionShape() != null) {
            boolean isDynamic = !(body.isStaticObject() || body.isKinematicObject());
            short collisionFilterGroup = isDynamic ? CollisionFilterGroups.DEFAULT_FILTER : CollisionFilterGroups.STATIC_FILTER;
            short collisionFilterMask = isDynamic ? CollisionFilterGroups.ALL_FILTER : (short) (CollisionFilterGroups.ALL_FILTER ^ CollisionFilterGroups.STATIC_FILTER);

            addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
        }
    }

    public void addRigidBody(RigidBody body, short group, short mask) {
        if (!body.isStaticOrKinematicObject()) {
            body.setGravity(gravity);
        }

        if (body.getCollisionShape() != null) {
            addCollisionObject(body, group, mask);
        }
    }

    public void updateActions(double timeStep) {
        for (ActionInterface action : actions) {
            action.updateAction(this, timeStep);
        }
    }

    protected void updateVehicles(double timeStep) {
        for (RaycastVehicle vehicle : vehicles) {
            vehicle.updateVehicle(timeStep);
        }
    }

    protected void updateActivationState(double timeStep) {
        Vector3d tmp = Stack.newVec();

        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null) {
                body.updateDeactivation(timeStep);

                if (body.wantsSleeping()) {
                    if (body.isStaticOrKinematicObject()) {
                        body.setActivationState(CollisionObject.ISLAND_SLEEPING);
                    } else {
                        if (body.getActivationState() == CollisionObject.ACTIVE_TAG) {
                            body.setActivationState(CollisionObject.WANTS_DEACTIVATION);
                        }
                        if (body.getActivationState() == CollisionObject.ISLAND_SLEEPING) {
                            tmp.set(0.0, 0.0, 0.0);
                            body.setAngularVelocity(tmp);
                            body.setLinearVelocity(tmp);
                        }
                    }
                } else {
                    if (body.getActivationState() != CollisionObject.DISABLE_DEACTIVATION) {
                        body.setActivationState(CollisionObject.ACTIVE_TAG);
                    }
                }
            }
        }
    }

    @Override
    public void addConstraint(TypedConstraint constraint, boolean disableCollisionsBetweenLinkedBodies) {
        constraints.add(constraint);
        if (disableCollisionsBetweenLinkedBodies) {
            constraint.getRigidBodyA().addConstraintRef(constraint);
            constraint.getRigidBodyB().addConstraintRef(constraint);
        }
    }

    @Override
    public void removeConstraint(TypedConstraint constraint) {
        constraints.remove(constraint);
        constraint.getRigidBodyA().removeConstraintRef(constraint);
        constraint.getRigidBodyB().removeConstraintRef(constraint);
    }

    @Override
    public void addAction(ActionInterface action) {
        actions.add(action);
    }

    @Override
    public void removeAction(ActionInterface action) {
        actions.remove(action);
    }

    @Override
    public void addVehicle(RaycastVehicle vehicle) {
        vehicles.add(vehicle);
    }

    @Override
    public void removeVehicle(RaycastVehicle vehicle) {
        vehicles.remove(vehicle);
    }

    private static int getConstraintIslandId(TypedConstraint lhs) {
        int islandId;

        CollisionObject rcolObj0 = lhs.getRigidBodyA();
        CollisionObject rcolObj1 = lhs.getRigidBodyB();
        islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
        return islandId;
    }

    private static class InplaceSolverIslandCallback extends SimulationIslandManager.IslandCallback {
        public ContactSolverInfo solverInfo;
        public ConstraintSolver solver;
        public ArrayList<TypedConstraint> sortedConstraints;
        public int numConstraints;
        public IDebugDraw debugDrawer;
        //public StackAlloc* m_stackAlloc;
        public Dispatcher dispatcher;

        public void init(ContactSolverInfo solverInfo, ConstraintSolver solver, ArrayList<TypedConstraint> sortedConstraints, int numConstraints, IDebugDraw debugDrawer, Dispatcher dispatcher) {
            this.solverInfo = solverInfo;
            this.solver = solver;
            this.sortedConstraints = sortedConstraints;
            this.numConstraints = numConstraints;
            this.debugDrawer = debugDrawer;
            this.dispatcher = dispatcher;
        }

        public void processIsland(ArrayList<CollisionObject> bodies, int numBodies, ArrayList<PersistentManifold> manifolds, int manifolds_offset, int numManifolds, int islandId) {
            if (islandId < 0) {
                // we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
                solver.solveGroup(bodies, numBodies, manifolds, manifolds_offset, numManifolds, sortedConstraints, 0, numConstraints, solverInfo, debugDrawer/*,m_stackAlloc*/, dispatcher);
            } else {
                // also add all non-contact constraints/joints for this island
                //ArrayList<TypedConstraint> startConstraint = null;
                int startConstraint_idx = -1;
                int numCurConstraints = 0;
                int i;

                // find the first constraint for this island
                for (i = 0; i < numConstraints; i++) {
                    if (getConstraintIslandId(sortedConstraints.get(i)) == islandId) {
                        startConstraint_idx = i;
                        break;
                    }
                }
                // count the number of constraints in this island
                for (; i < numConstraints; i++) {
                    if (getConstraintIslandId(sortedConstraints.get(i)) == islandId) {
                        numCurConstraints++;
                    }
                }

                // only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
                if ((numManifolds + numCurConstraints) > 0) {
                    solver.solveGroup(bodies, numBodies, manifolds, manifolds_offset, numManifolds, sortedConstraints, startConstraint_idx, numCurConstraints, solverInfo, debugDrawer/*,m_stackAlloc*/, dispatcher);
                }
            }
        }
    }

    private final ArrayList<TypedConstraint> sortedConstraints = new ArrayList<>();
    private final InplaceSolverIslandCallback solverCallback = new InplaceSolverIslandCallback();

    protected void solveConstraints(ContactSolverInfo solverInfo) {
        // sorted version of all btTypedConstraint, based on islandId
        sortedConstraints.clear();
        sortedConstraints.addAll(constraints);
		sortedConstraints.sort(sortConstraintOnIslandPredicate);

        ArrayList<TypedConstraint> constraintsPtr = getNumConstraints() != 0 ? sortedConstraints : null;

        solverCallback.init(solverInfo, constraintSolver, constraintsPtr, sortedConstraints.size(), debugDrawer, dispatcher1);

        constraintSolver.prepareSolve(getCollisionWorld().getNumCollisionObjects(), getCollisionWorld().getDispatcher().getNumManifolds());

        // solve all the constraints for this island
        islandManager.buildAndProcessIslands(getCollisionWorld().getDispatcher(), getCollisionWorld().getCollisionObjectArray(), solverCallback);

        constraintSolver.allSolved(solverInfo, debugDrawer);
    }

    protected void calculateSimulationIslands() {

        getSimulationIslandManager().updateActivationState(getCollisionWorld(), getCollisionWorld().getDispatcher());

        for (TypedConstraint constraint : constraints) {
            RigidBody colObj0 = constraint.getRigidBodyA();
            RigidBody colObj1 = constraint.getRigidBodyB();

            if (((colObj0 != null) && (!colObj0.isStaticOrKinematicObject())) &&
                    ((colObj1 != null) && (!colObj1.isStaticOrKinematicObject()))) {
                if (colObj0.isActive() || colObj1.isActive()) {
                    getSimulationIslandManager().getUnionFind().unite((colObj0).getIslandTag(), (colObj1).getIslandTag());
                }
            }
        }

        // Store the island id in each body
        getSimulationIslandManager().storeIslandActivationState(getCollisionWorld());

    }

    protected void integrateTransforms(double timeStep) {

        Vector3d tmp = Stack.newVec();
        Transform tmpTrans = Stack.newTrans();

        Transform predictedTrans = Stack.newTrans();
        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null) {
                body.setHitFraction(1.0);

                if (body.isActive() && (!body.isStaticOrKinematicObject())) {
                    body.predictIntegratedTransform(timeStep, predictedTrans);

                    tmp.sub(predictedTrans.origin, body.getWorldTransform(tmpTrans).origin);
                    double squareMotion = tmp.lengthSquared();

                    if (body.getCcdSquareMotionThreshold() != 0.0 && body.getCcdSquareMotionThreshold() < squareMotion) {

                        if (body.getCollisionShape().isConvex()) {
                            BulletStats.gNumClampedCcdMotions++;

                            ClosestNotMeConvexResultCallback sweepResults = new ClosestNotMeConvexResultCallback(body, body.getWorldTransform(tmpTrans).origin, predictedTrans.origin, getBroadphase().getOverlappingPairCache(), getDispatcher());
                            //ConvexShape convexShape = (ConvexShape)body.getCollisionShape();
                            SphereShape tmpSphere = new SphereShape(body.getCcdSweptSphereRadius()); //btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());

                            sweepResults.collisionFilterGroup = body.getBroadphaseProxy().collisionFilterGroup;
                            sweepResults.collisionFilterMask = body.getBroadphaseProxy().collisionFilterMask;

                            convexSweepTest(tmpSphere, body.getWorldTransform(tmpTrans), predictedTrans, sweepResults);
                            // JAVA NOTE: added closestHitFraction test to prevent objects being stuck
                            if (sweepResults.hasHit() && (sweepResults.closestHitFraction > 0.0001)) {
                                body.setHitFraction(sweepResults.closestHitFraction);
                                body.predictIntegratedTransform(timeStep * body.getHitFraction(), predictedTrans);
                                body.setHitFraction(0f);
                                //System.out.printf("clamped integration to hit fraction = %f\n", sweepResults.closestHitFraction);
                            }
                        }

                    }

                    body.proceedToTransform(predictedTrans);
                }
            }
        }
    }

    protected void predictUnconstraintMotion(double timeStep) {

        Transform tmpTrans = Stack.newTrans();

        for (CollisionObject colObj : collisionObjects) {
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null) {
                if (!body.isStaticOrKinematicObject()) {
                    if (body.isActive()) {
                        body.integrateVelocities(timeStep);
                        // damping
                        body.applyDamping(timeStep);

                        body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
                    }
                }
            }
        }
    }

    protected void startProfiling(double timeStep) {

    }

    protected void debugDrawSphere(double radius, Transform transform, Vector3d color) {
        Vector3d start = Stack.newVec(transform.origin);

        Vector3d xoffs = Stack.newVec();
        xoffs.set(radius, 0, 0);
        transform.basis.transform(xoffs);
        Vector3d yoffs = Stack.newVec();
        yoffs.set(0, radius, 0);
        transform.basis.transform(yoffs);
        Vector3d zoffs = Stack.newVec();
        zoffs.set(0, 0, radius);
        transform.basis.transform(zoffs);

        Vector3d tmp1 = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        // XY
        tmp1.sub(start, xoffs);
        tmp2.add(start, yoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.add(start, yoffs);
        tmp2.add(start, xoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.add(start, xoffs);
        tmp2.sub(start, yoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.sub(start, yoffs);
        tmp2.sub(start, xoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);

        // XZ
        tmp1.sub(start, xoffs);
        tmp2.add(start, zoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.add(start, zoffs);
        tmp2.add(start, xoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.add(start, xoffs);
        tmp2.sub(start, zoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.sub(start, zoffs);
        tmp2.sub(start, xoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);

        // YZ
        tmp1.sub(start, yoffs);
        tmp2.add(start, zoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.add(start, zoffs);
        tmp2.add(start, yoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.add(start, yoffs);
        tmp2.sub(start, zoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
        tmp1.sub(start, zoffs);
        tmp2.sub(start, yoffs);
        getDebugDrawer().drawLine(tmp1, tmp2, color);
    }

    public void debugDrawObject(Transform worldTransform, CollisionShape shape, Vector3d color) {
        Vector3d tmp = Stack.newVec();
        Vector3d tmp2 = Stack.newVec();

        // Draw a small simplex at the center of the object
        {
            Vector3d start = Stack.newVec(worldTransform.origin);

            tmp.set(1.0, 0.0, 0.0);
            worldTransform.basis.transform(tmp);
            tmp.add(start);
            tmp2.set(1.0, 0.0, 0.0);
            getDebugDrawer().drawLine(start, tmp, tmp2);

            tmp.set(0.0, 1.0, 0.0);
            worldTransform.basis.transform(tmp);
            tmp.add(start);
            tmp2.set(0.0, 1.0, 0.0);
            getDebugDrawer().drawLine(start, tmp, tmp2);

            tmp.set(0.0, 0.0, 1.0);
            worldTransform.basis.transform(tmp);
            tmp.add(start);
            tmp2.set(0.0, 0.0, 1.0);
            getDebugDrawer().drawLine(start, tmp, tmp2);
        }
    }

    @Override
    public void setConstraintSolver(ConstraintSolver solver) {
        ownsConstraintSolver = false;
        constraintSolver = solver;
    }

    @Override
    public ConstraintSolver getConstraintSolver() {
        return constraintSolver;
    }

    @Override
    public int getNumConstraints() {
        return constraints.size();
    }

    @Override
    public TypedConstraint getConstraint(int index) {
        return constraints.get(index);
    }

    // JAVA NOTE: not part of the original api
    @Override
    public int getNumActions() {
        return actions.size();
    }

    // JAVA NOTE: not part of the original api
    @Override
    public ActionInterface getAction(int index) {
        return actions.get(index);
    }

    public SimulationIslandManager getSimulationIslandManager() {
        return islandManager;
    }

    public CollisionWorld getCollisionWorld() {
        return this;
    }

	////////////////////////////////////////////////////////////////////////////

    private static final Comparator<TypedConstraint> sortConstraintOnIslandPredicate = (lhs, rhs) -> {
		int li = getConstraintIslandId(lhs);
		int ri = getConstraintIslandId(rhs);
		return Integer.compare(li, ri);
	};

    private static class ClosestNotMeConvexResultCallback extends ClosestConvexResultCallback {
        private final CollisionObject me;
		private final OverlappingPairCache pairCache;
        private final Dispatcher dispatcher;

        public ClosestNotMeConvexResultCallback(CollisionObject me, Vector3d fromA, Vector3d toA, OverlappingPairCache pairCache, Dispatcher dispatcher) {
            super(fromA, toA);
            this.me = me;
            this.pairCache = pairCache;
            this.dispatcher = dispatcher;
        }

        @Override
        public double addSingleResult(LocalConvexResult convexResult, boolean normalInWorldSpace) {
            if (convexResult.hitCollisionObject == me) {
                return 1.0;
            }

            Vector3d linVelA = Stack.newVec(), linVelB = Stack.newVec();
            linVelA.sub(convexToWorld, convexFromWorld);
            linVelB.set(0.0, 0.0, 0.0);//toB.getOrigin()-fromB.getOrigin();

            Vector3d relativeVelocity = Stack.newVec();
            relativeVelocity.sub(linVelA, linVelB);
            // don't report time of impact for motion away from the contact normal (or causes minor penetration)
			double allowedPenetration = 0.0;
			if (convexResult.hitNormalLocal.dot(relativeVelocity) >= -allowedPenetration) {
                return 1.0;
            }

            return super.addSingleResult(convexResult, normalInWorldSpace);
        }

        @Override
        public boolean needsCollision(BroadphaseProxy proxy0) {
            // don't collide with itself
            if (proxy0.clientObject == me) {
                return false;
            }

            // don't do CCD when the collision filters are not matching
            if (!super.needsCollision(proxy0)) {
                return false;
            }

            CollisionObject otherObj = (CollisionObject) proxy0.clientObject;

            // call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
            if (dispatcher.needsResponse(me, otherObj)) {
                // don't do CCD when there are already contact points (touching contact/penetration)
                ArrayList<PersistentManifold> manifoldArray = new ArrayList<>();
                BroadphasePair collisionPair = pairCache.findPair(me.getBroadphaseHandle(), proxy0);
                if (collisionPair != null) {
                    if (collisionPair.algorithm != null) {
                        //manifoldArray.resize(0);
                        collisionPair.algorithm.getAllContactManifolds(manifoldArray);
                        for (PersistentManifold manifold : manifoldArray) {
                            if (manifold.getNumContacts() > 0) {
                                return false;
                            }
                        }
                    }
                }
            }
            return true;
        }
    }

}
