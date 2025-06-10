package com.bulletphysics.dynamics

import com.bulletphysics.BulletGlobals
import com.bulletphysics.BulletStats
import com.bulletphysics.BulletStats.popProfile
import com.bulletphysics.BulletStats.pushProfile
import com.bulletphysics.collision.broadphase.*
import com.bulletphysics.collision.dispatch.CollisionConfiguration
import com.bulletphysics.collision.dispatch.CollisionObject
import com.bulletphysics.collision.dispatch.CollisionWorld
import com.bulletphysics.collision.dispatch.SimulationIslandManager
import com.bulletphysics.collision.dispatch.SimulationIslandManager.IslandCallback
import com.bulletphysics.collision.narrowphase.PersistentManifold
import com.bulletphysics.collision.shapes.SphereShape
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver
import com.bulletphysics.dynamics.constraintsolver.ContactSolverInfo
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint
import com.bulletphysics.dynamics.vehicle.RaycastVehicle
import com.bulletphysics.linearmath.*
import com.bulletphysics.util.ObjectArrayList
import cz.advel.stack.Stack
import java.util.function.ToIntFunction
import javax.vecmath.Vector3d
import kotlin.math.abs
import kotlin.math.min

/**
 * DiscreteDynamicsWorld provides discrete rigid body simulation.
 *
 * @author jezek2
 */
class DiscreteDynamicsWorld(
    dispatcher: Dispatcher,
    pairCache: BroadphaseInterface,
    override var constraintSolver: ConstraintSolver,
    collisionConfiguration: CollisionConfiguration
) : DynamicsWorld(dispatcher, pairCache, collisionConfiguration) {

    val simulationIslandManager = SimulationIslandManager()
    val constraints: ObjectArrayList<TypedConstraint> = ObjectArrayList<TypedConstraint>()
    val gravity: Vector3d = Vector3d(0.0, -10.0, 0.0)

    //for variable timesteps
    var localTime: Double = 1.0 / 60.0

    val vehicles = ObjectArrayList<RaycastVehicle>()
    val actions = ObjectArrayList<ActionInterface?>()

    fun saveKinematicState(timeStep: Double) {
        for (i in collisionObjectArray.indices) {
            val colObj = collisionObjectArray.getQuick(i)
            val body = RigidBody.upcast(colObj)
            if (body != null && body.activationState != CollisionObject.ISLAND_SLEEPING && body.isKinematicObject) {
                // to calculate velocities next frame
                body.saveKinematicState(timeStep)
            }
        }
    }

    override fun debugDrawWorld() {
        if (debugDrawer != null && (debugDrawer!!.getDebugMode() and DebugDrawModes.DRAW_CONTACT_POINTS) != 0) {
            val numManifolds = dispatcher.numManifolds
            val color = Stack.newVec()
            color.set(0.0, 0.0, 0.0)
            for (i in 0 until numManifolds) {
                val contactManifold = dispatcher.getManifoldByIndexInternal(i)

                //btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
                //btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
                val numContacts = contactManifold.getNumContacts()
                for (j in 0 until numContacts) {
                    val cp = contactManifold.getContactPoint(j)
                    debugDrawer!!.drawContactPoint(
                        cp.positionWorldOnB,
                        cp.normalWorldOnB,
                        cp.getDistance(),
                        cp.getLifeTime(),
                        color
                    )
                }
            }
        }

        if (debugDrawer != null && (debugDrawer!!.getDebugMode() and (DebugDrawModes.DRAW_WIREFRAME or DebugDrawModes.DRAW_AABB)) != 0) {
            var i: Int

            val tmpTrans = Stack.newTrans()
            val minAabb = Stack.newVec()
            val maxAabb = Stack.newVec()
            val colorvec = Stack.newVec()

            // todo: iterate over awake simulation islands!
            i = 0
            while (i < collisionObjectArray.size) {
                val colObj = collisionObjectArray.getQuick(i)
                if (debugDrawer != null && (debugDrawer!!.getDebugMode() and DebugDrawModes.DRAW_WIREFRAME) != 0) {
                    val color = Stack.newVec()
                    color.set(255.0, 255.0, 255.0)
                    when (colObj.activationState) {
                        CollisionObject.ACTIVE_TAG -> color.set(255.0, 255.0, 255.0)
                        CollisionObject.ISLAND_SLEEPING -> color.set(0.0, 255.0, 0.0)
                        CollisionObject.WANTS_DEACTIVATION -> color.set(0.0, 255.0, 255.0)
                        CollisionObject.DISABLE_DEACTIVATION -> color.set(255.0, 0.0, 0.0)
                        CollisionObject.DISABLE_SIMULATION -> color.set(255.0, 255.0, 0.0)
                        else -> {
                            color.set(255.0, 0.0, 0.0)
                        }
                    }

                    debugDrawObject(colObj.getWorldTransform(tmpTrans))
                }
                if (debugDrawer != null && (debugDrawer!!.getDebugMode() and DebugDrawModes.DRAW_AABB) != 0) {
                    colorvec.set(1.0, 0.0, 0.0)
                    colObj.collisionShape!!.getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb)
                    debugDrawer!!.drawAabb(minAabb, maxAabb, colorvec)
                }
                i++
            }

            val wheelColor = Stack.newVec()
            val wheelPosWS = Stack.newVec()
            val axle = Stack.newVec()
            val tmp = Stack.newVec()

            i = 0
            while (i < vehicles.size) {
                for (v in 0 until vehicles.getQuick(i).getNumWheels()) {
                    wheelColor.set(0.0, 255.0, 255.0)
                    if (vehicles.getQuick(i).getWheelInfo(v).raycastInfo.isInContact) {
                        wheelColor.set(0.0, 0.0, 255.0)
                    } else {
                        wheelColor.set(255.0, 0.0, 255.0)
                    }

                    wheelPosWS.set(vehicles.getQuick(i).getWheelInfo(v).worldTransform.origin)

                    axle.set(
                        vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis.getElement(
                            0,
                            vehicles.getQuick(i).getRightAxis()
                        ),
                        vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis.getElement(
                            1,
                            vehicles.getQuick(i).getRightAxis()
                        ),
                        vehicles.getQuick(i).getWheelInfo(v).worldTransform.basis.getElement(
                            2,
                            vehicles.getQuick(i).getRightAxis()
                        )
                    )


                    //m_vehicles[i]->getWheelInfo(v).m_raycastInfo.m_wheelAxleWS
                    //debug wheels (cylinders)
                    tmp.add(wheelPosWS, axle)
                    debugDrawer!!.drawLine(wheelPosWS, tmp, wheelColor)
                    debugDrawer!!.drawLine(
                        wheelPosWS,
                        vehicles.getQuick(i).getWheelInfo(v).raycastInfo.contactPointWS,
                        wheelColor
                    )
                }
                i++
            }

            if (debugDrawer != null && debugDrawer!!.getDebugMode() != 0) {
                i = 0
                while (i < actions.size) {
                    actions.getQuick(i)!!.debugDraw(debugDrawer!!)
                    i++
                }
            }
        }
    }

    override fun clearForces() {
        // todo: iterate over awake simulation islands!
        for (i in 0 until collisionObjectArray.size) {
            val colObj = collisionObjectArray.getQuick(i)

            val body = RigidBody.upcast(colObj)
            body?.clearForces()
        }
    }

    /**
     * Apply gravity, call this once per timestep.
     */
    fun applyGravity() {
        // todo: iterate over awake simulation islands!
        for (i in 0 until collisionObjectArray.size) {
            val colObj = collisionObjectArray.getQuick(i)

            val body = RigidBody.upcast(colObj)
            if (body != null && body.isActive) {
                body.applyGravity()
            }
        }
    }

    fun synchronizeMotionStates() {
        val interpolatedTransform = Stack.newTrans()

        val tmpTrans = Stack.newTrans()
        val tmpLinVel = Stack.newVec()
        val tmpAngVel = Stack.newVec()

        var stackPos: IntArray? = null
        // todo: iterate over awake simulation islands!
        for (i in 0 until collisionObjectArray.size) {
            val colObj = collisionObjectArray.getQuick(i)

            val body = RigidBody.upcast(colObj)
            if (body != null && body.getMotionState() != null && !body.isStaticOrKinematicObject) {
                // we need to call the update at least once, even for sleeping objects
                // otherwise the 'graphics' transform never updates properly
                // so todo: add 'dirty' flag
                stackPos = Stack.getPosition(stackPos)
                TransformUtil.integrateTransform(
                    body.getInterpolationWorldTransform(tmpTrans),
                    body.getInterpolationLinearVelocity(tmpLinVel),
                    body.getInterpolationAngularVelocity(tmpAngVel),
                    localTime * body.hitFraction, interpolatedTransform
                )
                body.getMotionState().setWorldTransform(interpolatedTransform)
                Stack.reset(stackPos)
            }
        }
        Stack.subTrans(2)
        Stack.subVec(2)

        if (debugDrawer != null && (debugDrawer!!.getDebugMode() and DebugDrawModes.DRAW_WIREFRAME) != 0) {
            for (i in vehicles.indices) {
                for (v in 0 until vehicles.getQuick(i).getNumWheels()) {
                    stackPos = Stack.getPosition(stackPos)
                    // synchronize the wheels with the (interpolated) chassis worldtransform
                    vehicles.getQuick(i).updateWheelTransform(v, true)
                    Stack.reset(stackPos)
                }
            }
        }
    }

    override fun stepSimulation(timeStep: Double, maxSubSteps: Int, fixedTimeStep: Double): Int {
        var maxSubSteps = maxSubSteps
        var fixedTimeStep = fixedTimeStep
        startProfiling()

        val t0 = System.nanoTime()

        pushProfile("stepSimulation")
        try {
            var numSimulationSubSteps = 0
            if (maxSubSteps != 0) {
                // fixed timestep with interpolation
                localTime += timeStep
                if (localTime >= fixedTimeStep) {
                    numSimulationSubSteps = (localTime / fixedTimeStep).toInt()
                    localTime -= numSimulationSubSteps * fixedTimeStep
                }
            } else {
                //variable timestep
                fixedTimeStep = timeStep
                localTime = timeStep
                if (abs(timeStep) >= BulletGlobals.FLT_EPSILON) {
                    numSimulationSubSteps = 1
                    maxSubSteps = 1
                }
            }

            // process some debugging flags
            if (debugDrawer != null) {
                BulletGlobals.setDeactivationDisabled((debugDrawer!!.getDebugMode() and DebugDrawModes.NO_DEACTIVATION) != 0)
            }
            if (numSimulationSubSteps != 0) {
                saveKinematicState(fixedTimeStep)

                applyGravity()

                // clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
                val clampedSimulationSteps = min(numSimulationSubSteps, maxSubSteps)

                for (i in 0 until clampedSimulationSteps) {
                    internalSingleStepSimulation(fixedTimeStep)
                    synchronizeMotionStates()
                }
            }

            synchronizeMotionStates()

            clearForces()

            //#ifndef BT_NO_PROFILE
            CProfileManager.incrementFrameCounter()

            //#endif //BT_NO_PROFILE
            return numSimulationSubSteps
        } finally {
            popProfile()

            BulletStats.stepSimulationTime = (System.nanoTime() - t0) / 1000000
        }
    }

    fun internalSingleStepSimulation(timeStep: Double) {
        pushProfile("internalSingleStepSimulation")
        try {
            // apply gravity, predict motion
            predictUnconstrainedMotion(timeStep)

            val dispatchInfo = dispatchInfo

            dispatchInfo.timeStep = timeStep
            dispatchInfo.stepCount = 0
            dispatchInfo.debugDraw = debugDrawer

            // perform collision detection
            performDiscreteCollisionDetection()

            calculateSimulationIslands()

            solverInfo.timeStep = timeStep

            // solve contact and other joint constraints
            solveConstraints(solverInfo)

            removeBrokenConstraints()

            //CallbackTriggers();

            // integrate transforms
            integrateTransforms(timeStep)

            // update vehicle simulation
            updateActions(timeStep)

            // update vehicle simulation
            updateVehicles(timeStep)

            updateActivationState(timeStep)

            internalTickCallback?.internalTick(this, timeStep)
        } finally {
            popProfile()
        }
    }

    private fun removeBrokenConstraints() {
        val callback = this.brokenConstraintCallback
        constraints.removeIf { constraint: TypedConstraint? ->
            if (callback != null && constraint!!.isBroken) {
                callback.onBrokenConstraint(constraint)
            }
            constraint!!.isBroken
        }
    }

    override fun setGravity(gravity: Vector3d) {
        this.gravity.set(gravity)
        var i = 0
        val l: Int = collisionObjectArray.size
        while (i < l) {
            val colObj = collisionObjectArray.getQuick(i)
            val body = RigidBody.upcast(colObj)
            body?.setGravity(gravity)
            i++
        }
    }

    override fun getGravity(out: Vector3d): Vector3d {
        out.set(gravity)
        return out
    }

    override fun removeRigidBody(body: RigidBody) {
        removeCollisionObject(body)
    }

    override fun addRigidBody(body: RigidBody) {
        if (!body.isStaticOrKinematicObject) {
            body.setGravity(gravity)
        }

        if (body.collisionShape != null) {
            val isDynamic = !(body.isStaticObject || body.isKinematicObject)
            val collisionFilterGroup =
                if (isDynamic) CollisionFilterGroups.DEFAULT_FILTER else CollisionFilterGroups.STATIC_FILTER
            val collisionFilterMask =
                if (isDynamic) CollisionFilterGroups.ALL_FILTER else (CollisionFilterGroups.ALL_FILTER.toInt() xor CollisionFilterGroups.STATIC_FILTER.toInt()).toShort()

            addCollisionObject(body, collisionFilterGroup, collisionFilterMask)
        }
    }

    @Suppress("unused")
    fun addRigidBody(body: RigidBody, group: Short, mask: Short) {
        if (!body.isStaticOrKinematicObject) {
            body.setGravity(gravity)
        }

        if (body.collisionShape != null) {
            addCollisionObject(body, group, mask)
        }
    }

    fun updateActions(timeStep: Double) {
        pushProfile("updateActions")
        try {
            for (i in actions.indices) {
                actions.getQuick(i)!!.updateAction(this, timeStep)
            }
        } finally {
            popProfile()
        }
    }

    fun updateVehicles(timeStep: Double) {
        pushProfile("updateVehicles")
        try {
            for (i in vehicles.indices) {
                val vehicle = vehicles.getQuick(i)
                vehicle.updateVehicle(timeStep)
            }
        } finally {
            popProfile()
        }
    }

    fun updateActivationState(timeStep: Double) {
        pushProfile("updateActivationState")
        try {
            var stackPos: IntArray? = null
            for (i in 0 until collisionObjectArray.size) {
                stackPos = Stack.getPosition(stackPos)
                val colObj = collisionObjectArray.getQuick(i)
                val body = RigidBody.upcast(colObj)
                if (body != null) {
                    body.updateDeactivation(timeStep)

                    if (body.wantsSleeping()) {
                        if (body.isStaticOrKinematicObject) {
                            body.setActivationStateMaybe(CollisionObject.ISLAND_SLEEPING)
                        } else {
                            if (body.activationState == CollisionObject.ACTIVE_TAG) {
                                body.setActivationStateMaybe(CollisionObject.WANTS_DEACTIVATION)
                            }
                            if (body.activationState == CollisionObject.ISLAND_SLEEPING) {
                                val zero = Stack.borrowVec()
                                zero.set(0.0, 0.0, 0.0)
                                body.setAngularVelocity(zero)
                                body.setLinearVelocity(zero)
                            }
                        }
                    } else {
                        if (body.activationState != CollisionObject.DISABLE_DEACTIVATION) {
                            body.setActivationStateMaybe(CollisionObject.ACTIVE_TAG)
                        }
                    }
                }
                Stack.reset(stackPos)
            }
        } finally {
            popProfile()
        }
    }

    override fun addConstraint(constraint: TypedConstraint, disableCollisionsBetweenLinkedBodies: Boolean) {
        constraints.add(constraint)
        if (disableCollisionsBetweenLinkedBodies) {
            constraint.rigidBodyA.addConstraintRef(constraint)
            constraint.rigidBodyB.addConstraintRef(constraint)
        }
    }

    override fun removeConstraint(constraint: TypedConstraint) {
        constraints.remove(constraint)
        constraint.rigidBodyA.removeConstraintRef(constraint)
        constraint.rigidBodyB.removeConstraintRef(constraint)
    }

    override fun addAction(action: ActionInterface) {
        actions.add(action)
    }

    override fun removeAction(action: ActionInterface) {
        actions.remove(action)
    }

    override fun addVehicle(vehicle: RaycastVehicle) {
        vehicles.add(vehicle)
    }

    override fun removeVehicle(vehicle: RaycastVehicle) {
        vehicles.remove(vehicle)
    }

    private class InplaceSolverIslandCallback : IslandCallback() {
        var solverInfo: ContactSolverInfo? = null
        var solver: ConstraintSolver? = null
        var sortedConstraints: ObjectArrayList<TypedConstraint>? = null
        var numConstraints: Int = 0
        var debugDrawer: IDebugDraw? = null

        //public StackAlloc* m_stackAlloc;
        var dispatcher: Dispatcher? = null

        fun init(
            solverInfo: ContactSolverInfo,
            solver: ConstraintSolver,
            sortedConstraints: ObjectArrayList<TypedConstraint>?,
            numConstraints: Int,
            debugDrawer: IDebugDraw?,
            dispatcher: Dispatcher
        ) {
            this.solverInfo = solverInfo
            this.solver = solver
            this.sortedConstraints = sortedConstraints
            this.numConstraints = numConstraints
            this.debugDrawer = debugDrawer
            this.dispatcher = dispatcher
        }

        override fun processIsland(
            bodies: ObjectArrayList<CollisionObject>, numBodies: Int,
            manifolds: ObjectArrayList<PersistentManifold>, manifoldsOffset: Int, numManifolds: Int,
            islandId: Int
        ) {
            if (islandId < 0) {
                // we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
                solver!!.solveGroup(
                    bodies, numBodies, manifolds, manifoldsOffset, numManifolds,
                    sortedConstraints, 0, numConstraints,
                    solverInfo!!, debugDrawer, dispatcher!!
                )
            } else {

                // also add all non-contact constraints/joints for this island
                //ObjectArrayList<TypedConstraint> startConstraint = null;
                var startConstraintIdx = -1
                var numCurConstraints = 0

                val sortedConstraints = sortedConstraints

                if (sortedConstraints != null) {
                    // find the first constraint for this island
                    for (i in 0 until numConstraints) {
                        if (getConstraintIslandId(sortedConstraints.getQuick(i)!!) == islandId) {
                            //startConstraint = &m_sortedConstraints[i];
                            //startConstraint = sortedConstraints.subList(i, sortedConstraints.getSize());
                            startConstraintIdx = i
                            break
                        }
                    }
                    // count the number of constraints in this island
                    for (i in 0 until numConstraints) {
                        if (getConstraintIslandId(sortedConstraints.getQuick(i)!!) == islandId) {
                            numCurConstraints++
                        }
                    }
                }

                // only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
                if (numManifolds + numCurConstraints > 0) {
                    solver!!.solveGroup(
                        bodies, numBodies, manifolds, manifoldsOffset, numManifolds, sortedConstraints,
                        startConstraintIdx, numCurConstraints, solverInfo!!, debugDrawer, dispatcher!!
                    )
                }
            }
        }
    }

    private val sortedConstraints = ObjectArrayList<TypedConstraint>()
    private val solverCallback = InplaceSolverIslandCallback()

    fun solveConstraints(solverInfo: ContactSolverInfo) {
        pushProfile("solveConstraints")
        try {
            // sorted version of all btTypedConstraint, based on islandId
            sortedConstraints.clear()
            for (i in constraints.indices) {
                sortedConstraints.add(constraints.getQuick(i))
            }
            MiscUtil.quickSort(sortedConstraints, sortConstraintOnIslandPredicate)

            val constraintsPtr = if (numConstraints != 0) sortedConstraints else null

            solverCallback.init(
                solverInfo,
                constraintSolver,
                constraintsPtr,
                sortedConstraints.size,
                debugDrawer,  /*,m_stackAlloc*/
                dispatcher
            )

            constraintSolver.prepareSolve(
                this.collisionWorld.numCollisionObjects,
                this.collisionWorld.dispatcher.numManifolds
            )

            // solve all the constraints for this island
            simulationIslandManager.buildAndProcessIslands(
                this.collisionWorld.dispatcher,
                this.collisionWorld.collisionObjectArray,
                solverCallback
            )

            constraintSolver.allSolved(solverInfo, debugDrawer /*, m_stackAlloc*/)
        } finally {
            popProfile()
        }
    }

    fun calculateSimulationIslands() {
        pushProfile("calculateSimulationIslands")
        try {
            this.simulationIslandManager.updateActivationState(this.collisionWorld, this.collisionWorld.dispatcher)

            var i = 0
            val l = constraints.size
            while (i < l) {
                val constraint = constraints.getQuick(i)

                val colObj0 = constraint.rigidBodyA
                val colObj1 = constraint.rigidBodyB

                if (((colObj0 != null) && (!colObj0.isStaticOrKinematicObject)) &&
                    ((colObj1 != null) && (!colObj1.isStaticOrKinematicObject))
                ) {
                    if (colObj0.isActive || colObj1.isActive) {
                        this.simulationIslandManager.unionFind
                            .combineIslands((colObj0).islandTag, (colObj1).islandTag)
                    }
                }
                i++
            }

            // Store the island id in each body
            this.simulationIslandManager.storeIslandActivationState(this.collisionWorld)
        } finally {
            popProfile()
        }
    }

    private val tmpSphere = SphereShape(1.0)
    private val sweepResults = ClosestNotMeConvexResultCallback()

    fun integrateTransforms(timeStep: Double) {
        pushProfile("integrateTransforms")
        try {
            val tmp = Stack.newVec()
            val tmpTrans = Stack.newTrans()
            val predictedTrans = Stack.newTrans()
            var stackPos: IntArray? = null
            for (i in 0 until collisionObjectArray.size) {
                val colObj = collisionObjectArray.getQuick(i)
                val body = RigidBody.upcast(colObj)
                if (body != null) {
                    body.hitFraction = 1.0
                    if (body.isActive && !body.isStaticOrKinematicObject) {
                        stackPos = Stack.getPosition(stackPos)

                        body.predictIntegratedTransform(timeStep, predictedTrans)

                        tmp.sub(predictedTrans.origin, body.getWorldTransform(tmpTrans).origin)
                        val squareMotion = tmp.lengthSquared()

                        if (body.ccdSquareMotionThreshold != 0.0 && body.ccdSquareMotionThreshold < squareMotion) {
                            pushProfile("CCD motion clamping")
                            try {
                                if (body.collisionShape!!.isConvex) {
                                    BulletStats.numClampedCcdMotions++

                                    val sweepResults = this.sweepResults
                                    sweepResults.init(
                                        body, body.getWorldTransform(tmpTrans).origin,
                                        predictedTrans.origin, broadphase.getOverlappingPairCache(), dispatcher
                                    )

                                    val tmpSphere = this.tmpSphere
                                    tmpSphere.margin = body.ccdSweptSphereRadius

                                    sweepResults.collisionFilterGroup = body.getBroadphaseProxy().collisionFilterGroup
                                    sweepResults.collisionFilterMask = body.getBroadphaseProxy().collisionFilterMask

                                    convexSweepTest(
                                        tmpSphere,
                                        body.getWorldTransform(tmpTrans),
                                        predictedTrans,
                                        sweepResults
                                    )
                                    // JAVA NOTE: added closestHitFraction test to prevent objects being stuck
                                    if (sweepResults.hasHit() && (sweepResults.closestHitFraction > 0.0001f)) {
                                        body.hitFraction = sweepResults.closestHitFraction
                                        body.predictIntegratedTransform(timeStep * body.hitFraction, predictedTrans)
                                        body.hitFraction = 0.0
                                    }
                                }
                            } finally {
                                popProfile()
                            }
                        }
                        body.proceedToTransform(predictedTrans)
                        Stack.reset(stackPos)
                    }
                }
            }
        } finally {
            Stack.subVec(1)
            Stack.subTrans(2)
            popProfile()
        }
    }

    fun predictUnconstrainedMotion(timeStep: Double) {
        pushProfile("predictUnconstrainedMotion")
        try {
            var stackPos: IntArray? = null
            val tmpTrans = Stack.newTrans()
            for (i in 0 until collisionObjectArray.size) {
                val colObj = collisionObjectArray.getQuick(i)
                val body = RigidBody.upcast(colObj)
                if (body != null && !body.isStaticOrKinematicObject && body.isActive) {
                    stackPos = Stack.getPosition(stackPos)
                    body.integrateVelocities(timeStep)
                    // damping
                    body.applyDamping(timeStep)
                    body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans))
                    Stack.reset(stackPos)
                }
            }
        } finally {
            Stack.subTrans(1)
            popProfile()
        }
    }

    fun startProfiling() {
        CProfileManager.reset()
    }

    fun debugDrawObject(worldTransform: Transform) {
        val tmp = Stack.newVec()
        val color = Stack.newVec()

        // Draw a small simplex at the center of the object
        val start = Stack.newVec(worldTransform.origin)

        tmp.set(1.0, 0.0, 0.0)
        worldTransform.basis.transform(tmp)
        tmp.add(start)
        color.set(1.0, 0.0, 0.0)
        debugDrawer!!.drawLine(start, tmp, color)

        tmp.set(0.0, 1.0, 0.0)
        worldTransform.basis.transform(tmp)
        tmp.add(start)
        color.set(0.0, 1.0, 0.0)
        debugDrawer!!.drawLine(start, tmp, color)

        tmp.set(0.0, 0.0, 1.0)
        worldTransform.basis.transform(tmp)
        tmp.add(start)
        color.set(0.0, 0.0, 1.0)
        debugDrawer!!.drawLine(start, tmp, color)

        Stack.subVec(3)
    }

    override val numConstraints: Int
        get() = constraints.size

    override fun getConstraint(index: Int): TypedConstraint? {
        return constraints.getQuick(index)
    }

    override val numActions: Int
        get() = actions.size

    override fun getAction(index: Int): ActionInterface? {
        return actions.getQuick(index)
    }

    val collisionWorld: CollisionWorld
        get() = this

    private class ClosestNotMeConvexResultCallback : ClosestConvexResultCallback() {
        private var me: CollisionObject? = null
        private var pairCache: OverlappingPairCache? = null
        private var dispatcher: Dispatcher? = null

        fun init(
            me: CollisionObject,
            fromA: Vector3d,
            toA: Vector3d,
            pairCache: OverlappingPairCache,
            dispatcher: Dispatcher
        ) {
            super.init(fromA, toA)
            this.me = me
            this.pairCache = pairCache
            this.dispatcher = dispatcher
        }

        public override fun addSingleResult(convexResult: LocalConvexResult, normalInWorldSpace: Boolean): Double {
            if (convexResult.hitCollisionObject === me) {
                return 1.0
            }

            val linVelA = Stack.newVec()
            val linVelB = Stack.newVec()
            linVelA.sub(convexToWorld, convexFromWorld)
            linVelB.set(0.0, 0.0, 0.0) //toB.getOrigin()-fromB.getOrigin();

            val relativeVelocity = Stack.newVec()
            relativeVelocity.sub(linVelA, linVelB)

            // don't report time of impact for motion away from the contact normal (or causes minor penetration)
            val allowedPenetration = 0.0
            val ignored = convexResult.hitNormalLocal.dot(relativeVelocity) >= -allowedPenetration

            Stack.subVec(3)
            if (ignored) return 1.0

            return super.addSingleResult(convexResult, normalInWorldSpace)
        }

        public override fun needsCollision(proxy0: BroadphaseProxy): Boolean {
            // don't collide with itself
            if (proxy0.clientObject === me) {
                return false
            }

            // don't do CCD when the collision filters are not matching
            if (!super.needsCollision(proxy0)) {
                return false
            }

            val otherObj = proxy0.clientObject as CollisionObject

            // call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
            if (!dispatcher!!.needsResponse(me!!, otherObj)) return true

            // don't do CCD when there are already contact points (touching contact/penetration)
            val manifoldArray = Stack.newList<PersistentManifold?>()
            val collisionPair = pairCache!!.findPair(me!!.broadphaseHandle!!, proxy0)
            if (collisionPair != null) {
                if (collisionPair.algorithm != null) {
                    //manifoldArray.resize(0);
                    collisionPair.algorithm.getAllContactManifolds(manifoldArray)
                    for (j in manifoldArray.indices) {
                        val manifold = manifoldArray.getQuick(j)
                        if (manifold.getNumContacts() > 0) {
                            // cleanup
                            manifoldArray.clear()
                            Stack.subList(1)
                            return false
                        }
                    }
                    // cleanup
                    manifoldArray.clear()
                }
            }
            Stack.subList(1)

            return true
        }
    }

    companion object {
        private fun getConstraintIslandId(lhs: TypedConstraint): Int {
            val islandId: Int

            val colObj0: CollisionObject = lhs.rigidBodyA
            val colObj1: CollisionObject = lhs.rigidBodyB
            islandId = if (colObj0.islandTag >= 0) colObj0.islandTag else colObj1.islandTag
            return islandId
        }

        /** ///////////////////////////////////////////////////////////////////////// */
        private val sortConstraintOnIslandPredicate: Comparator<TypedConstraint?> =
            Comparator.comparingInt<TypedConstraint?>(ToIntFunction { lhs: TypedConstraint? ->
                Companion.getConstraintIslandId(
                    lhs!!
                )
            })
    }
}
