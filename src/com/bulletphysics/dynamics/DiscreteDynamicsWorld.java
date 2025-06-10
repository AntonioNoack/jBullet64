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
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.constraintsolver.*;
import com.bulletphysics.dynamics.vehicle.RaycastVehicle;
import com.bulletphysics.linearmath.*;
import com.bulletphysics.util.ObjectArrayList;
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
    protected final ObjectArrayList<TypedConstraint> constraints = new ObjectArrayList<>();
    protected final Vector3d gravity = new Vector3d(0.0, -10.0, 0.0);

    //for variable timesteps
    protected double localTime = 1.0 / 60.0;

    protected boolean ownsIslandManager;
    protected boolean ownsConstraintSolver;

    public ObjectArrayList<RaycastVehicle> vehicles = new ObjectArrayList<>();

    public ObjectArrayList<ActionInterface> actions = new ObjectArrayList<>();

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
        for (int i = 0; i < collisionObjects.size(); i++) {
            CollisionObject colObj = collisionObjects.getQuick(i);
            RigidBody body = RigidBody.upcast(colObj);
            if (body != null && body.getActivationState() != CollisionObject.ISLAND_SLEEPING && body.isKinematicObject()) {
                // to calculate velocities next frame
                body.saveKinematicState(timeStep);
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
                //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
                //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

                int numContacts = contactManifold.getNumContacts();
                for (int j = 0; j < numContacts; j++) {
                    ManifoldPoint cp = contactManifold.getContactPoint(j);
                    getDebugDrawer().drawContactPoint(cp.positionWorldOnB, cp.normalWorldOnB, cp.getDistance(), cp.getLifeTime(), color);
                }
            }
        }

        if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & (DebugDrawModes.DRAW_WIREFRAME | DebugDrawModes.DRAW_AABB)) != 0) {
            int i;

            Transform tmpTrans = Stack.newTrans();
            Vector3d minAabb = Stack.newVec();
            Vector3d maxAabb = Stack.newVec();
            Vector3d colorvec = Stack.newVec();

            // todo: iterate over awake simulation islands!
            for (i = 0; i < collisionObjects.size(); i++) {
                CollisionObject colObj = collisionObjects.getQuick(i);
                if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_WIREFRAME) != 0) {
                    Vector3d color = Stack.newVec();
                    color.set(255f, 255f, 255f);
                    switch (colObj.getActivationState()) {
                        case CollisionObject.ACTIVE_TAG:
                            color.set(255f, 255f, 255f);
                            break;
                        case CollisionObject.ISLAND_SLEEPING:
                            color.set(0.0, 255f, 0.0);
                            break;
                        case CollisionObject.WANTS_DEACTIVATION:
                            color.set(0.0, 255f, 255f);
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

                    debugDrawObject(colObj.getWorldTransform(tmpTrans));
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
                for (int v = 0; v < vehicles.getQuick(i).getNumWheels(); v++) {
                    wheelColor.set(0, 255, 255);
                    if (vehicles.getQuick(i).getWheelInfo(v).raycastInfo.isInContact) {
                        wheelColor.set(0, 0, 255);
                    } else {
                        wheelColor.set(255, 0, 255);
                    }

                    wheelPosWS.set(vehicles.getQuick(i).getWheelInfo(v).worldTransform.origin);

                    axle.set(
                            vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis.getElement(0, vehicles.getQuick(i).getRightAxis()),
                            vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis.getElement(1, vehicles.getQuick(i).getRightAxis()),
                            vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis.getElement(2, vehicles.getQuick(i).getRightAxis()));


                    //m_vehicles[i]->getWheelInfo(v).m_raycastInfo.m_wheelAxleWS
                    //debug wheels (cylinders)
                    tmp.add(wheelPosWS, axle);
                    debugDrawer.drawLine(wheelPosWS, tmp, wheelColor);
                    debugDrawer.drawLine(wheelPosWS, vehicles.getQuick(i).getWheelInfo(v).raycastInfo.contactPointWS, wheelColor);
                }
            }

            if (getDebugDrawer() != null && getDebugDrawer().getDebugMode() != 0) {
                for (i = 0; i < actions.size(); i++) {
                    actions.getQuick(i).debugDraw(debugDrawer);
                }
            }
        }
    }

    @Override
    public void clearForces() {
        // todo: iterate over awake simulation islands!
        for (int i = 0; i < collisionObjects.size(); i++) {
            CollisionObject colObj = collisionObjects.getQuick(i);

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
        for (int i = 0; i < collisionObjects.size(); i++) {
            CollisionObject colObj = collisionObjects.getQuick(i);

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

        int[] stackPos = null;
        // todo: iterate over awake simulation islands!
        for (int i = 0; i < collisionObjects.size(); i++) {
            CollisionObject colObj = collisionObjects.getQuick(i);

            RigidBody body = RigidBody.upcast(colObj);
            if (body != null && body.getMotionState() != null && !body.isStaticOrKinematicObject()) {
                // we need to call the update at least once, even for sleeping objects
                // otherwise the 'graphics' transform never updates properly
                // so todo: add 'dirty' flag
                stackPos = Stack.getPosition(stackPos);
                TransformUtil.integrateTransform(
                        body.getInterpolationWorldTransform(tmpTrans),
                        body.getInterpolationLinearVelocity(tmpLinVel),
                        body.getInterpolationAngularVelocity(tmpAngVel),
                        localTime * body.hitFraction, interpolatedTransform);
                body.getMotionState().setWorldTransform(interpolatedTransform);
                Stack.reset(stackPos);
            }
        }
        Stack.subTrans(2);
        Stack.subVec(2);

        if (getDebugDrawer() != null && (getDebugDrawer().getDebugMode() & DebugDrawModes.DRAW_WIREFRAME) != 0) {
            for (int i = 0; i < vehicles.size(); i++) {
                for (int v = 0; v < vehicles.getQuick(i).getNumWheels(); v++) {
                    stackPos = Stack.getPosition(stackPos);
                    // synchronize the wheels with the (interpolated) chassis worldtransform
                    vehicles.getQuick(i).updateWheelTransform(v, true);
                    Stack.reset(stackPos);
                }
            }
        }
    }

    @Override
    public int stepSimulation(double timeStep, int maxSubSteps, double fixedTimeStep) {
        startProfiling();

        long t0 = System.nanoTime();

        BulletStats.pushProfile("stepSimulation");
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
                if (Math.abs(timeStep) >= BulletGlobals.FLT_EPSILON) {
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

            //#ifndef BT_NO_PROFILE
            CProfileManager.incrementFrameCounter();
            //#endif //BT_NO_PROFILE

            return numSimulationSubSteps;
        } finally {
            BulletStats.popProfile();

            BulletStats.stepSimulationTime = (System.nanoTime() - t0) / 1000000;
        }
    }

    protected void internalSingleStepSimulation(double timeStep) {
        BulletStats.pushProfile("internalSingleStepSimulation");
        try {
            // apply gravity, predict motion
            predictUnconstrainedMotion(timeStep);

            DispatcherInfo dispatchInfo = getDispatchInfo();

            dispatchInfo.timeStep = timeStep;
            dispatchInfo.stepCount = 0;
            dispatchInfo.debugDraw = getDebugDrawer();

            // perform collision detection
            performDiscreteCollisionDetection();

            calculateSimulationIslands();

            solverInfo.timeStep = timeStep;

            // solve contact and other joint constraints
            solveConstraints(solverInfo);

            removeBrokenConstraints();

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
        } finally {
            BulletStats.popProfile();
        }
    }

    private void removeBrokenConstraints() {
        BrokenConstraintCallback callback = this.brokenConstraintCallback;
        constraints.removeIf(constraint -> {
            if (callback != null && constraint.isBroken()) {
                callback.onBrokenConstraint(constraint);
            }
            // constraint might be repaired in the callback
            return constraint.isBroken();
        });
    }

    @Override
    public void setGravity(Vector3d gravity) {
        this.gravity.set(gravity);
        for (int i = 0, l = collisionObjects.size(); i < l; i++) {
            CollisionObject colObj = collisionObjects.getQuick(i);
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

    @SuppressWarnings("unused")
    public void addRigidBody(RigidBody body, short group, short mask) {
        if (!body.isStaticOrKinematicObject()) {
            body.setGravity(gravity);
        }

        if (body.getCollisionShape() != null) {
            addCollisionObject(body, group, mask);
        }
    }

    public void updateActions(double timeStep) {
        BulletStats.pushProfile("updateActions");
        try {
            for (int i = 0; i < actions.size(); i++) {
                actions.getQuick(i).updateAction(this, timeStep);
            }
        } finally {
            BulletStats.popProfile();
        }
    }

    protected void updateVehicles(double timeStep) {
        BulletStats.pushProfile("updateVehicles");
        try {
            for (int i = 0; i < vehicles.size(); i++) {
                RaycastVehicle vehicle = vehicles.getQuick(i);
                vehicle.updateVehicle(timeStep);
            }
        } finally {
            BulletStats.popProfile();
        }
    }

    protected void updateActivationState(double timeStep) {
        BulletStats.pushProfile("updateActivationState");
        try {
            int[] stackPos = null;
            for (int i = 0; i < collisionObjects.size(); i++) {
                stackPos = Stack.getPosition(stackPos);
                CollisionObject colObj = collisionObjects.getQuick(i);
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
                                Vector3d zero = Stack.borrowVec();
                                zero.set(0.0, 0.0, 0.0);
                                body.setAngularVelocity(zero);
                                body.setLinearVelocity(zero);
                            }
                        }
                    } else {
                        if (body.getActivationState() != CollisionObject.DISABLE_DEACTIVATION) {
                            body.setActivationState(CollisionObject.ACTIVE_TAG);
                        }
                    }
                }
                Stack.reset(stackPos);
            }
        } finally {
            BulletStats.popProfile();
        }
    }

    @Override
    public void addConstraint(TypedConstraint constraint, boolean disableCollisionsBetweenLinkedBodies) {
        constraints.add(constraint);
        if (disableCollisionsBetweenLinkedBodies) {
            constraint.rigidBodyA.addConstraintRef(constraint);
            constraint.rigidBodyB.addConstraintRef(constraint);
        }
    }

    @Override
    public void removeConstraint(TypedConstraint constraint) {
        constraints.remove(constraint);
        constraint.rigidBodyA.removeConstraintRef(constraint);
        constraint.rigidBodyB.removeConstraintRef(constraint);
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

        CollisionObject colObj0 = lhs.rigidBodyA;
        CollisionObject colObj1 = lhs.rigidBodyB;
        islandId = colObj0.islandTag >= 0 ? colObj0.islandTag : colObj1.islandTag;
        return islandId;
    }

    private static class InplaceSolverIslandCallback extends SimulationIslandManager.IslandCallback {
        public ContactSolverInfo solverInfo;
        public ConstraintSolver solver;
        public ObjectArrayList<TypedConstraint> sortedConstraints;
        public int numConstraints;
        public IDebugDraw debugDrawer;
        //public StackAlloc* m_stackAlloc;
        public Dispatcher dispatcher;

        public void init(ContactSolverInfo solverInfo, ConstraintSolver solver, ObjectArrayList<TypedConstraint> sortedConstraints, int numConstraints, IDebugDraw debugDrawer, Dispatcher dispatcher) {
            this.solverInfo = solverInfo;
            this.solver = solver;
            this.sortedConstraints = sortedConstraints;
            this.numConstraints = numConstraints;
            this.debugDrawer = debugDrawer;
            this.dispatcher = dispatcher;
        }

        public void processIsland(ObjectArrayList<CollisionObject> bodies, int numBodies, ObjectArrayList<PersistentManifold> manifolds, int manifoldsOffset, int numManifolds, int islandId) {
            if (islandId < 0) {
                // we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
                solver.solveGroup(bodies, numBodies, manifolds, manifoldsOffset, numManifolds, sortedConstraints, 0, numConstraints, solverInfo, debugDrawer/*,m_stackAlloc*/, dispatcher);
            } else {
                // also add all non-contact constraints/joints for this island
                //ObjectArrayList<TypedConstraint> startConstraint = null;
                int startConstraintIdx = -1;
                int numCurConstraints = 0;
                int i;

                // find the first constraint for this island
                for (i = 0; i < numConstraints; i++) {
                    if (getConstraintIslandId(sortedConstraints.getQuick(i)) == islandId) {
                        //startConstraint = &m_sortedConstraints[i];
                        //startConstraint = sortedConstraints.subList(i, sortedConstraints.size());
                        startConstraintIdx = i;
                        break;
                    }
                }
                // count the number of constraints in this island
                for (; i < numConstraints; i++) {
                    if (getConstraintIslandId(sortedConstraints.getQuick(i)) == islandId) {
                        numCurConstraints++;
                    }
                }

                // only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
                if (numManifolds + numCurConstraints > 0) {
                    solver.solveGroup(bodies, numBodies, manifolds, manifoldsOffset, numManifolds, sortedConstraints,
                            startConstraintIdx, numCurConstraints, solverInfo, debugDrawer, dispatcher);
                }
            }
        }
    }

    private final ObjectArrayList<TypedConstraint> sortedConstraints = new ObjectArrayList<>();
    private final InplaceSolverIslandCallback solverCallback = new InplaceSolverIslandCallback();

    protected void solveConstraints(ContactSolverInfo solverInfo) {
        BulletStats.pushProfile("solveConstraints");
        try {
            // sorted version of all btTypedConstraint, based on islandId
            sortedConstraints.clear();
            for (int i = 0; i < constraints.size(); i++) {
                sortedConstraints.add(constraints.getQuick(i));
            }
            MiscUtil.quickSort(sortedConstraints, sortConstraintOnIslandPredicate);

            ObjectArrayList<TypedConstraint> constraintsPtr = getNumConstraints() != 0 ? sortedConstraints : null;

            solverCallback.init(solverInfo, constraintSolver, constraintsPtr, sortedConstraints.size(), debugDrawer/*,m_stackAlloc*/, dispatcher1);

            constraintSolver.prepareSolve(getCollisionWorld().getNumCollisionObjects(), getCollisionWorld().getDispatcher().getNumManifolds());

            // solve all the constraints for this island
            islandManager.buildAndProcessIslands(getCollisionWorld().getDispatcher(), getCollisionWorld().getCollisionObjectArray(), solverCallback);

            constraintSolver.allSolved(solverInfo, debugDrawer/*, m_stackAlloc*/);
        } finally {
            BulletStats.popProfile();
        }
    }

    protected void calculateSimulationIslands() {
        BulletStats.pushProfile("calculateSimulationIslands");
        try {
            getSimulationIslandManager().updateActivationState(getCollisionWorld(), getCollisionWorld().getDispatcher());

            for (int i = 0, l = constraints.size(); i < l; i++) {
                TypedConstraint constraint = constraints.getQuick(i);

                RigidBody colObj0 = constraint.rigidBodyA;
                RigidBody colObj1 = constraint.rigidBodyB;

                if (((colObj0 != null) && (!colObj0.isStaticOrKinematicObject())) &&
                        ((colObj1 != null) && (!colObj1.isStaticOrKinematicObject()))) {
                    if (colObj0.isActive() || colObj1.isActive()) {
                        getSimulationIslandManager().getUnionFind().combineIslands((colObj0).islandTag, (colObj1).islandTag);
                    }
                }
            }

            // Store the island id in each body
            getSimulationIslandManager().storeIslandActivationState(getCollisionWorld());
        } finally {
            BulletStats.popProfile();
        }
    }

    private final SphereShape tmpSphere = new SphereShape(1.0);
    private final ClosestNotMeConvexResultCallback sweepResults = new ClosestNotMeConvexResultCallback();

    protected void integrateTransforms(double timeStep) {
        BulletStats.pushProfile("integrateTransforms");
        try {
            Vector3d tmp = Stack.newVec();
            Transform tmpTrans = Stack.newTrans();
            Transform predictedTrans = Stack.newTrans();
            int[] stackPos = null;
            for (int i = 0; i < collisionObjects.size(); i++) {
                CollisionObject colObj = collisionObjects.getQuick(i);
                RigidBody body = RigidBody.upcast(colObj);
                if (body != null) {
                    body.hitFraction = 1.0;
                    if (body.isActive() && !body.isStaticOrKinematicObject()) {
                        stackPos = Stack.getPosition(stackPos);

                        body.predictIntegratedTransform(timeStep, predictedTrans);

                        tmp.sub(predictedTrans.origin, body.getWorldTransform(tmpTrans).origin);
                        double squareMotion = tmp.lengthSquared();

                        if (body.getCcdSquareMotionThreshold() != 0.0 && body.getCcdSquareMotionThreshold() < squareMotion) {
                            BulletStats.pushProfile("CCD motion clamping");
                            try {
                                if (body.getCollisionShape().isConvex()) {
                                    BulletStats.gNumClampedCcdMotions++;

                                    ClosestNotMeConvexResultCallback sweepResults = this.sweepResults;
                                    sweepResults.init(
                                            body, body.getWorldTransform(tmpTrans).origin,
                                            predictedTrans.origin, getBroadphase().getOverlappingPairCache(), getDispatcher()
                                    );

                                    SphereShape tmpSphere = this.tmpSphere;
                                    tmpSphere.setRadius(body.ccdSweptSphereRadius);

                                    sweepResults.collisionFilterGroup = body.getBroadphaseProxy().collisionFilterGroup;
                                    sweepResults.collisionFilterMask = body.getBroadphaseProxy().collisionFilterMask;

                                    convexSweepTest(tmpSphere, body.getWorldTransform(tmpTrans), predictedTrans, sweepResults);
                                    // JAVA NOTE: added closestHitFraction test to prevent objects being stuck
                                    if (sweepResults.hasHit() && (sweepResults.closestHitFraction > 0.0001f)) {
                                        body.hitFraction = sweepResults.closestHitFraction;
                                        body.predictIntegratedTransform(timeStep * body.hitFraction, predictedTrans);
                                        body.hitFraction = 0.0;
                                    }
                                }
                            } finally {
                                BulletStats.popProfile();
                            }
                        }
                        body.proceedToTransform(predictedTrans);
                        Stack.reset(stackPos);
                    }
                }
            }
        } finally {
            Stack.subVec(1);
            Stack.subTrans(2);
            BulletStats.popProfile();
        }
    }

    protected void predictUnconstrainedMotion(double timeStep) {
        BulletStats.pushProfile("predictUnconstrainedMotion");
        try {
            int[] stackPos = null;
            Transform tmpTrans = Stack.newTrans();
            for (int i = 0; i < collisionObjects.size(); i++) {
                CollisionObject colObj = collisionObjects.getQuick(i);
                RigidBody body = RigidBody.upcast(colObj);
                if (body != null && !body.isStaticOrKinematicObject() && body.isActive()) {
                    stackPos = Stack.getPosition(stackPos);
                    body.integrateVelocities(timeStep);
                    // damping
                    body.applyDamping(timeStep);
                    body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
                    Stack.reset(stackPos);
                }
            }
        } finally {
            Stack.subTrans(1);
            BulletStats.popProfile();
        }
    }

    protected void startProfiling() {
        CProfileManager.reset();
    }

    public void debugDrawObject(Transform worldTransform) {
        Vector3d tmp = Stack.newVec();
        Vector3d color = Stack.newVec();

        // Draw a small simplex at the center of the object
        Vector3d start = Stack.newVec(worldTransform.origin);

        tmp.set(1.0, 0.0, 0.0);
        worldTransform.basis.transform(tmp);
        tmp.add(start);
        color.set(1.0, 0.0, 0.0);
        getDebugDrawer().drawLine(start, tmp, color);

        tmp.set(0.0, 1.0, 0.0);
        worldTransform.basis.transform(tmp);
        tmp.add(start);
        color.set(0.0, 1.0, 0.0);
        getDebugDrawer().drawLine(start, tmp, color);

        tmp.set(0.0, 0.0, 1.0);
        worldTransform.basis.transform(tmp);
        tmp.add(start);
        color.set(0.0, 0.0, 1.0);
        getDebugDrawer().drawLine(start, tmp, color);

        Stack.subVec(3);
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
        return constraints.getQuick(index);
    }

    // JAVA NOTE: not part of the original api
    @Override
    public int getNumActions() {
        return actions.size();
    }

    // JAVA NOTE: not part of the original api
    @Override
    public ActionInterface getAction(int index) {
        return actions.getQuick(index);
    }

    public SimulationIslandManager getSimulationIslandManager() {
        return islandManager;
    }

    public CollisionWorld getCollisionWorld() {
        return this;
    }

    /// /////////////////////////////////////////////////////////////////////////

    private static final Comparator<TypedConstraint> sortConstraintOnIslandPredicate =
            Comparator.comparingInt(DiscreteDynamicsWorld::getConstraintIslandId);

    private static class ClosestNotMeConvexResultCallback extends ClosestConvexResultCallback {

        private CollisionObject me;
        private OverlappingPairCache pairCache;
        private Dispatcher dispatcher;

        void init(CollisionObject me, Vector3d fromA, Vector3d toA, OverlappingPairCache pairCache, Dispatcher dispatcher) {
            super.init(fromA, toA);
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
            boolean ignored = convexResult.hitNormalLocal.dot(relativeVelocity) >= -allowedPenetration;

            Stack.subVec(3);
            if (ignored) return 1.0;

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
            if (!dispatcher.needsResponse(me, otherObj)) return true;

            // don't do CCD when there are already contact points (touching contact/penetration)
            ObjectArrayList<PersistentManifold> manifoldArray = Stack.newList();
            BroadphasePair collisionPair = pairCache.findPair(me.getBroadphaseHandle(), proxy0);
            if (collisionPair != null) {
                if (collisionPair.algorithm != null) {
                    //manifoldArray.resize(0);
                    collisionPair.algorithm.getAllContactManifolds(manifoldArray);
                    for (int j = 0; j < manifoldArray.size(); j++) {
                        PersistentManifold manifold = manifoldArray.getQuick(j);
                        if (manifold.getNumContacts() > 0) {
                            // cleanup
                            manifoldArray.clear();
                            Stack.subList(1);
                            return false;
                        }
                    }
                    // cleanup
                    manifoldArray.clear();
                }
            }
            Stack.subList(1);

            return true;
        }
    }

}
