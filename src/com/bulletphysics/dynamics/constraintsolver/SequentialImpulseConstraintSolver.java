package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.MiscUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import static com.bulletphysics.util.Packing.*;

/**
 * SequentialImpulseConstraintSolver uses a Propagation Method and Sequentially applies impulses.
 * The approach is the 3D version of Erin Catto's GDC 2006 tutorial. See <a href="http://www.gphysics.com">GPhysics.com</a><p>
 * <p>
 * Although Sequential Impulse is more intuitive, it is mathematically equivalent to Projected
 * Successive Overrelaxation (iterative LCP).<p>
 * <p>
 * Applies impulses for combined restitution and penetration recovery and to simulate friction.
 *
 * @author jezek2
 */
public class SequentialImpulseConstraintSolver extends ConstraintSolver {

    private static final int MAX_CONTACT_SOLVER_TYPES = ContactConstraintEnum.MAX_CONTACT_SOLVER_TYPES.ordinal();

    private static final int SEQUENTIAL_IMPULSE_MAX_SOLVER_POINTS = 16384;
    private final long[] gOrder = new long[SEQUENTIAL_IMPULSE_MAX_SOLVER_POINTS];

    /// /////////////////////////////////////////////////////////////////////////

    private final ObjectPool<SolverBody> bodiesPool = ObjectPool.get(SolverBody.class);
    private final ObjectPool<SolverConstraint> constraintsPool = ObjectPool.get(SolverConstraint.class);
    private final ObjectPool<JacobianEntry> jacobiansPool = ObjectPool.get(JacobianEntry.class);

    private final ObjectArrayList<SolverBody> tmpSolverBodyPool = new ObjectArrayList<>();
    private final ObjectArrayList<SolverConstraint> tmpSolverConstraintPool = new ObjectArrayList<>();
    private final ObjectArrayList<SolverConstraint> tmpSolverFrictionConstraintPool = new ObjectArrayList<>();
    private final IntArrayList orderTmpConstraintPool = new IntArrayList();
    private final IntArrayList orderFrictionConstraintPool = new IntArrayList();

    protected final ContactSolverFunc[][] contactDispatch = new ContactSolverFunc[MAX_CONTACT_SOLVER_TYPES][MAX_CONTACT_SOLVER_TYPES];
    protected final ContactSolverFunc[][] frictionDispatch = new ContactSolverFunc[MAX_CONTACT_SOLVER_TYPES][MAX_CONTACT_SOLVER_TYPES];

    // btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
    protected long btSeed2 = 0L;

    public SequentialImpulseConstraintSolver() {
        BulletGlobals.setContactDestroyedCallback(userPersistentData -> {
            assert (userPersistentData != null);
            // ConstraintPersistentData cpd = (ConstraintPersistentData) userPersistentData;
            //btAlignedFree(cpd);
            //printf("totalCpd = %i. DELETED Ptr %x\n",totalCpd,userPersistentData);
            return true;
        });

        // initialize default friction/contact funcs
        int i, j;
        for (i = 0; i < MAX_CONTACT_SOLVER_TYPES; i++) {
            for (j = 0; j < MAX_CONTACT_SOLVER_TYPES; j++) {
                contactDispatch[i][j] = ContactConstraint.resolveSingleCollision;
                frictionDispatch[i][j] = ContactConstraint.resolveSingleFriction;
            }
        }
    }

    /**
     * See ODE: adam's all-int straightforward(?) dRandInt (0..n-1)
     */
    public int randInt2(int n) {

        // seems good; xor-fold and modulus
        btSeed2 = (1664525L * btSeed2 + 1013904223L);
        long r = btSeed2;

        // note: probably more aggressive than it needs to be -- might be
        //       able to get away without one or two of the innermost branches.
        if (n <= 0x10000) {
            r ^= (r >>> 16);
            if (n <= 0x100) {
                r ^= (r >>> 8);
                if (n <= 0x10) {
                    r ^= (r >>> 4);
                    if (n <= 0x4) {
                        r ^= (r >>> 2);
                        if (n <= 0x2) {
                            r ^= (r >>> 1);
                        }
                    }
                }
            }
        }

        // TODO: check modulo C vs Java mismatch
        return (int) Math.abs(r % n);
    }

    private void initSolverBody(SolverBody solverBody, CollisionObject collisionObject) {
        RigidBody rb = RigidBody.upcast(collisionObject);
        if (rb != null) {
            rb.getAngularVelocity(solverBody.angularVelocity);
            solverBody.centerOfMassPosition.set(collisionObject.getWorldTransform(Stack.newTrans()).origin);
            solverBody.friction = collisionObject.friction;
            solverBody.invMass = rb.inverseMass;
            rb.getLinearVelocity(solverBody.linearVelocity);
            solverBody.originalBody = rb;
            solverBody.angularFactor = rb.getAngularFactor();
        } else {
            solverBody.angularVelocity.set(0.0, 0.0, 0.0);
            solverBody.centerOfMassPosition.set(collisionObject.getWorldTransform(Stack.newTrans()).origin);
            solverBody.friction = collisionObject.friction;
            solverBody.invMass = 0.0;
            solverBody.linearVelocity.set(0.0, 0.0, 0.0);
            solverBody.originalBody = null;
            solverBody.angularFactor = 1.0;
        }

        solverBody.pushVelocity.set(0.0, 0.0, 0.0);
        solverBody.turnVelocity.set(0.0, 0.0, 0.0);
    }

    private double restitutionCurve(double rel_vel, double restitution) {
        return restitution * -rel_vel;
    }

    private void resolveSplitPenetrationImpulseCacheFriendly(
            SolverBody body1,
            SolverBody body2,
            SolverConstraint contactConstraint,
            ContactSolverInfo solverInfo) {

        if (contactConstraint.penetration < solverInfo.splitImpulsePenetrationThreshold) {
            BulletStats.numSplitImpulseRecoveries++;
            double normalImpulse;

            //  Optimized version of projected relative velocity, use precomputed cross products with normal
            //      body1.getVelocityInLocalPoint(contactConstraint.m_rel_posA,vel1);
            //      body2.getVelocityInLocalPoint(contactConstraint.m_rel_posB,vel2);
            //      btVector3 vel = vel1 - vel2;
            //      btScalar  rel_vel = contactConstraint.m_contactNormal.dot(vel);

            double rel_vel;
            double vel1Dotn = contactConstraint.contactNormal.dot(body1.pushVelocity) + contactConstraint.relPos1CrossNormal.dot(body1.turnVelocity);
            double vel2Dotn = contactConstraint.contactNormal.dot(body2.pushVelocity) + contactConstraint.relPos2CrossNormal.dot(body2.turnVelocity);

            rel_vel = vel1Dotn - vel2Dotn;

            double positionalError = -contactConstraint.penetration * solverInfo.erp2 / solverInfo.timeStep;
            //      btScalar positionalError = contactConstraint.m_penetration;

            double velocityError = contactConstraint.restitution - rel_vel;// * damping;

            double penetrationImpulse = positionalError * contactConstraint.jacDiagABInv;
            double velocityImpulse = velocityError * contactConstraint.jacDiagABInv;
            normalImpulse = penetrationImpulse + velocityImpulse;

            // See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
            double oldNormalImpulse = contactConstraint.appliedPushImpulse;
            double sum = oldNormalImpulse + normalImpulse;
            contactConstraint.appliedPushImpulse = Math.max(0.0, sum);

            normalImpulse = contactConstraint.appliedPushImpulse - oldNormalImpulse;

            Vector3d tmp = Stack.newVec();
            tmp.scale(body1.invMass, contactConstraint.contactNormal);
            body1.internalApplyPushImpulse(tmp, contactConstraint.angularComponentA, normalImpulse);

            tmp.scale(body2.invMass, contactConstraint.contactNormal);
            body2.internalApplyPushImpulse(tmp, contactConstraint.angularComponentB, -normalImpulse);
            Stack.subVec(1);
        }
    }

    /**
     * velocity + friction
     * response  between two dynamic objects with friction
     */
    private void resolveSingleCollisionCombinedCacheFriendly(
            SolverBody body1,
            SolverBody body2,
            SolverConstraint contactConstraint,
            ContactSolverInfo solverInfo) {

        // Optimized version of projected relative velocity, use precomputed cross products with normal
        // body1.getVelocityInLocalPoint(contactConstraint.m_rel_posA,vel1);
        // body2.getVelocityInLocalPoint(contactConstraint.m_rel_posB,vel2);
        // btVector3 vel = vel1 - vel2;
        // btScalar  rel_vel = contactConstraint.m_contactNormal.dot(vel);

        double rel_vel;
        double vel1Dotn = contactConstraint.contactNormal.dot(body1.linearVelocity) + contactConstraint.relPos1CrossNormal.dot(body1.angularVelocity);
        double vel2Dotn = contactConstraint.contactNormal.dot(body2.linearVelocity) + contactConstraint.relPos2CrossNormal.dot(body2.angularVelocity);

        rel_vel = vel1Dotn - vel2Dotn;

        double normalImpulse = getNormalImpulse(contactConstraint, solverInfo, rel_vel);

        // See Erin Catto's GDC 2006 paper: Clamp the accumulated impulse
        double oldNormalImpulse = contactConstraint.appliedImpulse;
        double sum = oldNormalImpulse + normalImpulse;
        contactConstraint.appliedImpulse = Math.max(0.0, sum);

        normalImpulse = contactConstraint.appliedImpulse - oldNormalImpulse;

        Vector3d tmp = Stack.newVec();
        tmp.scale(body1.invMass, contactConstraint.contactNormal);
        body1.internalApplyImpulse(tmp, contactConstraint.angularComponentA, normalImpulse);

        tmp.scale(body2.invMass, contactConstraint.contactNormal);
        body2.internalApplyImpulse(tmp, contactConstraint.angularComponentB, -normalImpulse);
        Stack.subVec(1);
    }

    private static double getNormalImpulse(SolverConstraint contactConstraint, ContactSolverInfo solverInfo, double rel_vel) {
        double positionalError = 0.0;
        if (!solverInfo.splitImpulse || (contactConstraint.penetration > solverInfo.splitImpulsePenetrationThreshold)) {
            positionalError = -contactConstraint.penetration * solverInfo.erp / solverInfo.timeStep;
        }

        double velocityError = contactConstraint.restitution - rel_vel;// * damping;

        double penetrationImpulse = positionalError * contactConstraint.jacDiagABInv;
        double velocityImpulse = velocityError * contactConstraint.jacDiagABInv;
        return penetrationImpulse + velocityImpulse;
    }

    private void resolveSingleFrictionCacheFriendly(
            SolverBody body1, SolverBody body2, SolverConstraint contactConstraint,
            double appliedNormalImpulse
    ) {
        double combinedFriction = contactConstraint.friction;

        double limit = appliedNormalImpulse * combinedFriction;

        if (appliedNormalImpulse > 0.0) { //friction

            double rel_vel;
            double vel1Dotn = contactConstraint.contactNormal.dot(body1.linearVelocity) + contactConstraint.relPos1CrossNormal.dot(body1.angularVelocity);
            double vel2Dotn = contactConstraint.contactNormal.dot(body2.linearVelocity) + contactConstraint.relPos2CrossNormal.dot(body2.angularVelocity);
            rel_vel = vel1Dotn - vel2Dotn;

            // calculate j that moves us to zero relative velocity
            double j1 = -rel_vel * contactConstraint.jacDiagABInv;
            double oldTangentImpulse = contactConstraint.appliedImpulse;
            contactConstraint.appliedImpulse = oldTangentImpulse + j1;

            if (limit < contactConstraint.appliedImpulse) {
                contactConstraint.appliedImpulse = limit;
            } else {
                if (contactConstraint.appliedImpulse < -limit) {
                    contactConstraint.appliedImpulse = -limit;
                }
            }
            j1 = contactConstraint.appliedImpulse - oldTangentImpulse;


            Vector3d tmp = Stack.newVec();
            tmp.scale(body1.invMass, contactConstraint.contactNormal);
            body1.internalApplyImpulse(tmp, contactConstraint.angularComponentA, j1);

            tmp.scale(body2.invMass, contactConstraint.contactNormal);
            body2.internalApplyImpulse(tmp, contactConstraint.angularComponentB, -j1);
            Stack.subVec(1);
        }
    }

    protected void addFrictionConstraint(
            Vector3d normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex,
            ManifoldPoint cp, Vector3d relPos1, Vector3d relPos2,
            CollisionObject colObj0, CollisionObject colObj1, double relaxation
    ) {
        RigidBody body0 = RigidBody.upcast(colObj0);
        RigidBody body1 = RigidBody.upcast(colObj1);

        SolverConstraint solverConstraint = constraintsPool.get();
        tmpSolverFrictionConstraintPool.add(solverConstraint);

        solverConstraint.contactNormal.set(normalAxis);

        solverConstraint.solverBodyIdA = solverBodyIdA;
        solverConstraint.solverBodyIdB = solverBodyIdB;
        solverConstraint.constraintType = SolverConstraintType.SOLVER_FRICTION_1D;
        solverConstraint.frictionIndex = frictionIndex;

        solverConstraint.friction = cp.combinedFriction;
        solverConstraint.originalContactPoint = null;

        solverConstraint.appliedImpulse = 0.0;
        solverConstraint.appliedPushImpulse = 0.0;
        solverConstraint.penetration = 0.0;

        Vector3d ftorqueAxis1 = Stack.newVec();
        Matrix3d tmpMat = Stack.newMat();


        ftorqueAxis1.cross(relPos1, solverConstraint.contactNormal);
        solverConstraint.relPos1CrossNormal.set(ftorqueAxis1);
        if (body0 != null) {
            solverConstraint.angularComponentA.set(ftorqueAxis1);
            body0.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentA);
        } else {
            solverConstraint.angularComponentA.set(0.0, 0.0, 0.0);
        }


        ftorqueAxis1.cross(relPos2, solverConstraint.contactNormal);
        solverConstraint.relPos2CrossNormal.set(ftorqueAxis1);
        if (body1 != null) {
            solverConstraint.angularComponentB.set(ftorqueAxis1);
            body1.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentB);
        } else {
            solverConstraint.angularComponentB.set(0.0, 0.0, 0.0);
        }


        Vector3d vec = Stack.newVec();
        double denom0 = 0.0;
        double denom1 = 0.0;
        if (body0 != null) {
            vec.cross(solverConstraint.angularComponentA, relPos1);
            denom0 = body0.inverseMass + normalAxis.dot(vec);
        }
        if (body1 != null) {
            vec.cross(solverConstraint.angularComponentB, relPos2);
            denom1 = body1.inverseMass + normalAxis.dot(vec);
        }

        solverConstraint.jacDiagABInv = relaxation / (denom0 + denom1);

        Stack.subVec(2);
        Stack.subMat(1);
    }

    @SuppressWarnings("UnusedReturnValue")
    public double solveGroupCacheFriendlySetup(
            ObjectArrayList<PersistentManifold> manifoldPtr,
            int manifoldOffset, int numManifolds, ObjectArrayList<TypedConstraint> constraints, int constraintsOffset,
            int numConstraints, ContactSolverInfo infoGlobal) {

        BulletStats.pushProfile("solveGroupCacheFriendlySetup");
        try {

            if ((numConstraints + numManifolds) == 0) {
                return 0.0;
            }
            PersistentManifold manifold;
            CollisionObject colObj0, colObj1;

            Transform tmpTrans = Stack.newTrans();

            Vector3d relPos1 = Stack.newVec();
            Vector3d relPos2 = Stack.newVec();

            Vector3d pos1 = Stack.newVec();
            Vector3d pos2 = Stack.newVec();
            Vector3d vel = Stack.newVec();
            Vector3d torqueAxis0 = Stack.newVec();
            Vector3d torqueAxis1 = Stack.newVec();
            Vector3d vel1 = Stack.newVec();
            Vector3d vel2 = Stack.newVec();
            Vector3d vec = Stack.newVec();
            Vector3d tmp = Stack.newVec();

            Matrix3d tmpMat = Stack.newMat();

            for (int i = 0; i < numManifolds; i++) {
                manifold = manifoldPtr.getQuick(manifoldOffset + i);
                colObj0 = (CollisionObject) manifold.getBody0();
                colObj1 = (CollisionObject) manifold.getBody1();

                int solverBodyIdA = -1;
                int solverBodyIdB = -1;

                if (manifold.getNumContacts() != 0) {
                    if (colObj0.islandTag >= 0) {
                        if (colObj0.companionId >= 0) {
                            // body has already been converted
                            solverBodyIdA = colObj0.companionId;
                        } else {
                            solverBodyIdA = tmpSolverBodyPool.size();
                            SolverBody solverBody = bodiesPool.get();
                            tmpSolverBodyPool.add(solverBody);
                            initSolverBody(solverBody, colObj0);
                            colObj0.companionId = solverBodyIdA;
                        }
                    } else {
                        // create a static body
                        solverBodyIdA = tmpSolverBodyPool.size();
                        SolverBody solverBody = bodiesPool.get();
                        tmpSolverBodyPool.add(solverBody);
                        initSolverBody(solverBody, colObj0);
                    }

                    if (colObj1.islandTag >= 0) {
                        if (colObj1.companionId >= 0) {
                            solverBodyIdB = colObj1.companionId;
                        } else {
                            solverBodyIdB = tmpSolverBodyPool.size();
                            SolverBody solverBody = bodiesPool.get();
                            tmpSolverBodyPool.add(solverBody);
                            initSolverBody(solverBody, colObj1);
                            colObj1.companionId = solverBodyIdB;
                        }
                    } else {
                        // create a static body
                        solverBodyIdB = tmpSolverBodyPool.size();
                        SolverBody solverBody = bodiesPool.get();
                        tmpSolverBodyPool.add(solverBody);
                        initSolverBody(solverBody, colObj1);
                    }
                }

                double relaxation;

                for (int j = 0; j < manifold.getNumContacts(); j++) {

                    ManifoldPoint cp = manifold.getContactPoint(j);

                    if (cp.getDistance() <= 0.0) {
                        cp.getPositionWorldOnA(pos1);
                        cp.getPositionWorldOnB(pos2);

                        relPos1.sub(pos1, colObj0.getWorldTransform(tmpTrans).origin);
                        relPos2.sub(pos2, colObj1.getWorldTransform(tmpTrans).origin);

                        relaxation = 1.0;
                        double rel_vel;

                        int frictionIndex = tmpSolverConstraintPool.size();

                        {
                            SolverConstraint solverConstraint = constraintsPool.get();
                            tmpSolverConstraintPool.add(solverConstraint);
                            RigidBody rb0 = RigidBody.upcast(colObj0);
                            RigidBody rb1 = RigidBody.upcast(colObj1);

                            solverConstraint.solverBodyIdA = solverBodyIdA;
                            solverConstraint.solverBodyIdB = solverBodyIdB;
                            solverConstraint.constraintType = SolverConstraintType.SOLVER_CONTACT_1D;

                            solverConstraint.originalContactPoint = cp;

                            torqueAxis0.cross(relPos1, cp.normalWorldOnB);

                            if (rb0 != null) {
                                solverConstraint.angularComponentA.set(torqueAxis0);
                                rb0.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentA);
                            } else {
                                solverConstraint.angularComponentA.set(0.0, 0.0, 0.0);
                            }

                            torqueAxis1.cross(relPos2, cp.normalWorldOnB);

                            if (rb1 != null) {
                                solverConstraint.angularComponentB.set(torqueAxis1);
                                rb1.getInvInertiaTensorWorld(tmpMat).transform(solverConstraint.angularComponentB);
                            } else {
                                solverConstraint.angularComponentB.set(0.0, 0.0, 0.0);
                            }

                            {
                                //#ifdef COMPUTE_IMPULSE_DENOM
                                //btScalar denom0 = rb0->computeImpulseDenominator(pos1,cp.m_normalWorldOnB);
                                //btScalar denom1 = rb1->computeImpulseDenominator(pos2,cp.m_normalWorldOnB);
                                //#else
                                double denom0 = 0.0;
                                double denom1 = 0.0;
                                if (rb0 != null) {
                                    vec.cross(solverConstraint.angularComponentA, relPos1);
                                    denom0 = rb0.inverseMass + cp.normalWorldOnB.dot(vec);
                                }
                                if (rb1 != null) {
                                    vec.cross(solverConstraint.angularComponentB, relPos2);
                                    denom1 = rb1.inverseMass + cp.normalWorldOnB.dot(vec);
                                }
                                //#endif //COMPUTE_IMPULSE_DENOM

                                solverConstraint.jacDiagABInv = relaxation / (denom0 + denom1);
                            }

                            solverConstraint.contactNormal.set(cp.normalWorldOnB);
                            solverConstraint.relPos1CrossNormal.cross(relPos1, cp.normalWorldOnB);
                            solverConstraint.relPos2CrossNormal.cross(relPos2, cp.normalWorldOnB);

                            if (rb0 != null) {
                                rb0.getVelocityInLocalPoint(relPos1, vel1);
                            } else {
                                vel1.set(0.0, 0.0, 0.0);
                            }

                            if (rb1 != null) {
                                rb1.getVelocityInLocalPoint(relPos2, vel2);
                            } else {
                                vel2.set(0.0, 0.0, 0.0);
                            }

                            vel.sub(vel1, vel2);

                            rel_vel = cp.normalWorldOnB.dot(vel);

                            solverConstraint.penetration = Math.min(cp.getDistance() + infoGlobal.linearSlop, 0.0);
                            //solverConstraint.m_penetration = cp.getDistance();

                            solverConstraint.friction = cp.combinedFriction;
                            solverConstraint.restitution = restitutionCurve(rel_vel, cp.combinedRestitution);
                            if (solverConstraint.restitution <= 0.0) {
                                solverConstraint.restitution = 0.0;
                            }

                            double penVel = -solverConstraint.penetration / infoGlobal.timeStep;

                            if (solverConstraint.restitution > penVel) {
                                solverConstraint.penetration = 0.0;
                            }

                            // warm starting (or zero if disabled)
                            if ((infoGlobal.solverMode & SolverMode.SOLVER_USE_WARMSTARTING) != 0) {
                                solverConstraint.appliedImpulse = cp.appliedImpulse * infoGlobal.warmstartingFactor;
                                if (rb0 != null) {
                                    tmp.scale(rb0.inverseMass, solverConstraint.contactNormal);
                                    tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdA).internalApplyImpulse(tmp, solverConstraint.angularComponentA, solverConstraint.appliedImpulse);
                                }
                                if (rb1 != null) {
                                    tmp.scale(rb1.inverseMass, solverConstraint.contactNormal);
                                    tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdB).internalApplyImpulse(tmp, solverConstraint.angularComponentB, -solverConstraint.appliedImpulse);
                                }
                            } else {
                                solverConstraint.appliedImpulse = 0.0;
                            }

                            solverConstraint.appliedPushImpulse = 0.0;

                            solverConstraint.frictionIndex = tmpSolverFrictionConstraintPool.size();
                            if (!cp.lateralFrictionInitialized) {
                                cp.lateralFrictionDir1.scale(rel_vel, cp.normalWorldOnB);
                                cp.lateralFrictionDir1.sub(vel, cp.lateralFrictionDir1);

                                double lat_rel_vel = cp.lateralFrictionDir1.lengthSquared();
                                if (lat_rel_vel > BulletGlobals.FLT_EPSILON)//0.0)
                                {
                                    cp.lateralFrictionDir1.scale(1.0 / Math.sqrt(lat_rel_vel));
                                    addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
                                    cp.lateralFrictionDir2.cross(cp.lateralFrictionDir1, cp.normalWorldOnB);
                                    cp.lateralFrictionDir2.normalize(); //??
                                    addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
                                } else {
                                    // re-calculate friction direction every frame, todo: check if this is really needed

                                    TransformUtil.planeSpace1(cp.normalWorldOnB, cp.lateralFrictionDir1, cp.lateralFrictionDir2);
                                    addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
                                    addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
                                }
                                cp.lateralFrictionInitialized = true;

                            } else {
                                addFrictionConstraint(cp.lateralFrictionDir1, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
                                addFrictionConstraint(cp.lateralFrictionDir2, solverBodyIdA, solverBodyIdB, frictionIndex, cp, relPos1, relPos2, colObj0, colObj1, relaxation);
                            }

                            {
                                SolverConstraint frictionConstraint1 = tmpSolverFrictionConstraintPool.getQuick(solverConstraint.frictionIndex);
                                if ((infoGlobal.solverMode & SolverMode.SOLVER_USE_WARMSTARTING) != 0) {
                                    frictionConstraint1.appliedImpulse = cp.appliedImpulseLateral1 * infoGlobal.warmstartingFactor;
                                    if (rb0 != null) {
                                        tmp.scale(rb0.inverseMass, frictionConstraint1.contactNormal);
                                        tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdA).internalApplyImpulse(tmp, frictionConstraint1.angularComponentA, frictionConstraint1.appliedImpulse);
                                    }
                                    if (rb1 != null) {
                                        tmp.scale(rb1.inverseMass, frictionConstraint1.contactNormal);
                                        tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdB).internalApplyImpulse(tmp, frictionConstraint1.angularComponentB, -frictionConstraint1.appliedImpulse);
                                    }
                                } else {
                                    frictionConstraint1.appliedImpulse = 0.0;
                                }
                            }
                            {
                                SolverConstraint frictionConstraint2 = tmpSolverFrictionConstraintPool.getQuick(solverConstraint.frictionIndex + 1);
                                if ((infoGlobal.solverMode & SolverMode.SOLVER_USE_WARMSTARTING) != 0) {
                                    frictionConstraint2.appliedImpulse = cp.appliedImpulseLateral2 * infoGlobal.warmstartingFactor;
                                    if (rb0 != null) {
                                        tmp.scale(rb0.inverseMass, frictionConstraint2.contactNormal);
                                        tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdA).internalApplyImpulse(tmp, frictionConstraint2.angularComponentA, frictionConstraint2.appliedImpulse);
                                    }
                                    if (rb1 != null) {
                                        tmp.scale(rb1.inverseMass, frictionConstraint2.contactNormal);
                                        tmpSolverBodyPool.getQuick(solverConstraint.solverBodyIdB).internalApplyImpulse(tmp, frictionConstraint2.angularComponentB, -frictionConstraint2.appliedImpulse);
                                    }
                                } else {
                                    frictionConstraint2.appliedImpulse = 0.0;
                                }
                            }
                        }
                    }
                }
            }

            for (int j = 0; j < numConstraints; j++) {
                TypedConstraint constraint = constraints.getQuick(constraintsOffset + j);
                constraint.buildJacobian();
            }

            int numConstraintPool = tmpSolverConstraintPool.size();
            int numFrictionPool = tmpSolverFrictionConstraintPool.size();

            MiscUtil.resize(orderTmpConstraintPool, numConstraintPool, 0);
            MiscUtil.resize(orderFrictionConstraintPool, numFrictionPool, 0);

            for (int j = 0; j < numConstraintPool; j++) {
                orderTmpConstraintPool.set(j, j);
            }
            for (int j = 0; j < numFrictionPool; j++) {
                orderFrictionConstraintPool.set(j, j);
            }

            Stack.subTrans(1);
            Stack.subMat(1);
            Stack.subVec(11);

            return 0.0;
        } finally {
            BulletStats.popProfile();
        }
    }

    @SuppressWarnings("UnusedReturnValue")
    public double solveGroupCacheFriendlyIterations(
            ObjectArrayList<TypedConstraint> constraints, int constraintsOffset, int numConstraints,
            ContactSolverInfo infoGlobal
    ) {
        BulletStats.pushProfile("solveGroupCacheFriendlyIterations");
        try {
            int numConstraintPool = tmpSolverConstraintPool.size();
            int numFrictionPool = tmpSolverFrictionConstraintPool.size();

            // should traverse the contacts random order...
            int iteration;
            int[] stackPos = null;
            for (iteration = 0; iteration < infoGlobal.numIterations; iteration++) {

                int j;
                if ((infoGlobal.solverMode & SolverMode.SOLVER_RANDMIZE_ORDER) != 0) {
                    if ((iteration & 7) == 0) {
                        for (j = 0; j < numConstraintPool; ++j) {
                            int tmp = orderTmpConstraintPool.get(j);
                            int swapi = randInt2(j + 1);
                            orderTmpConstraintPool.set(j, orderTmpConstraintPool.get(swapi));
                            orderTmpConstraintPool.set(swapi, tmp);
                        }

                        for (j = 0; j < numFrictionPool; ++j) {
                            int tmp = orderFrictionConstraintPool.get(j);
                            int swapi = randInt2(j + 1);
                            orderFrictionConstraintPool.set(j, orderFrictionConstraintPool.get(swapi));
                            orderFrictionConstraintPool.set(swapi, tmp);
                        }
                    }
                }

                for (int k = 0; k < numConstraints; k++) {
                    stackPos = Stack.getPosition(stackPos);
                    TypedConstraint constraint = constraints.getQuick(constraintsOffset + k);
                    // todo: use solver bodies, so we don't need to copy from/to btRigidBody

                    if ((constraint.rigidBodyA.islandTag >= 0) && (constraint.rigidBodyA.companionId >= 0)) {
                        tmpSolverBodyPool.getQuick(constraint.rigidBodyA.companionId).writebackVelocity();
                    }
                    if ((constraint.rigidBodyB.islandTag >= 0) && (constraint.rigidBodyB.companionId >= 0)) {
                        tmpSolverBodyPool.getQuick(constraint.rigidBodyB.companionId).writebackVelocity();
                    }

                    constraint.solveConstraint(infoGlobal.timeStep);

                    if ((constraint.rigidBodyA.islandTag >= 0) && (constraint.rigidBodyA.companionId >= 0)) {
                        tmpSolverBodyPool.getQuick(constraint.rigidBodyA.companionId).readVelocity();
                    }
                    if ((constraint.rigidBodyB.islandTag >= 0) && (constraint.rigidBodyB.companionId >= 0)) {
                        tmpSolverBodyPool.getQuick(constraint.rigidBodyB.companionId).readVelocity();
                    }
                    Stack.reset(stackPos);
                }


                int numPoolConstraints = tmpSolverConstraintPool.size();
                for (int k = 0; k < numPoolConstraints; k++) {
                    stackPos = Stack.getPosition(stackPos);
                    SolverConstraint solveManifold = tmpSolverConstraintPool.getQuick(orderTmpConstraintPool.get(k));
                    resolveSingleCollisionCombinedCacheFriendly(tmpSolverBodyPool.getQuick(solveManifold.solverBodyIdA),
                            tmpSolverBodyPool.getQuick(solveManifold.solverBodyIdB), solveManifold, infoGlobal);
                    Stack.reset(stackPos);
                }


                int numFrictionPoolConstraints = tmpSolverFrictionConstraintPool.size();
                for (int k = 0; k < numFrictionPoolConstraints; k++) {
                    stackPos = Stack.getPosition(stackPos);
                    SolverConstraint solveManifold = tmpSolverFrictionConstraintPool.getQuick(orderFrictionConstraintPool.get(k));

                    double totalImpulse = tmpSolverConstraintPool.getQuick(solveManifold.frictionIndex).appliedImpulse +
                            tmpSolverConstraintPool.getQuick(solveManifold.frictionIndex).appliedPushImpulse;

                    resolveSingleFrictionCacheFriendly(tmpSolverBodyPool.getQuick(solveManifold.solverBodyIdA),
                            tmpSolverBodyPool.getQuick(solveManifold.solverBodyIdB), solveManifold,
                            totalImpulse);
                    Stack.reset(stackPos);
                }
            }

            if (infoGlobal.splitImpulse) {
                for (iteration = 0; iteration < infoGlobal.numIterations; iteration++) {
                    int numPoolConstraints = tmpSolverConstraintPool.size();
                    for (int j = 0; j < numPoolConstraints; j++) {
                        stackPos = Stack.getPosition(stackPos);
                        SolverConstraint solveManifold = tmpSolverConstraintPool.getQuick(orderTmpConstraintPool.get(j));
                        resolveSplitPenetrationImpulseCacheFriendly(tmpSolverBodyPool.getQuick(solveManifold.solverBodyIdA),
                                tmpSolverBodyPool.getQuick(solveManifold.solverBodyIdB), solveManifold, infoGlobal);
                        Stack.reset(stackPos);
                    }
                }
            }

            return 0.0;
        } finally {
            BulletStats.popProfile();
        }
    }

    public double solveGroupCacheFriendly(
            ObjectArrayList<PersistentManifold> manifoldPtr, int manifoldOffset, int numManifolds,
            ObjectArrayList<TypedConstraint> constraints, int constraintsOffset, int numConstraints,
            ContactSolverInfo infoGlobal) {

        solveGroupCacheFriendlySetup(manifoldPtr, manifoldOffset, numManifolds, constraints, constraintsOffset, numConstraints, infoGlobal /*, stackAlloc*/);
        solveGroupCacheFriendlyIterations(constraints, constraintsOffset, numConstraints, infoGlobal /*, stackAlloc*/);

        int numPoolConstraints = tmpSolverConstraintPool.size();
        for (int j = 0; j < numPoolConstraints; j++) {

            SolverConstraint solveManifold = tmpSolverConstraintPool.getQuick(j);
            ManifoldPoint pt = (ManifoldPoint) solveManifold.originalContactPoint;
            assert (pt != null);
            pt.appliedImpulse = solveManifold.appliedImpulse;
            pt.appliedImpulseLateral1 = tmpSolverFrictionConstraintPool.getQuick(solveManifold.frictionIndex).appliedImpulse;
            pt.appliedImpulseLateral2 = tmpSolverFrictionConstraintPool.getQuick(solveManifold.frictionIndex + 1).appliedImpulse;

            // do a callback here?
        }

        if (infoGlobal.splitImpulse) {
            for (int i = 0; i < tmpSolverBodyPool.size(); i++) {
                tmpSolverBodyPool.getQuick(i).writebackVelocity(infoGlobal.timeStep);
                bodiesPool.release(tmpSolverBodyPool.getQuick(i));
            }
        } else {
            for (int i = 0; i < tmpSolverBodyPool.size(); i++) {
                tmpSolverBodyPool.getQuick(i).writebackVelocity();
                bodiesPool.release(tmpSolverBodyPool.getQuick(i));
            }
        }

        tmpSolverBodyPool.clear();
        for (int i = 0; i < tmpSolverConstraintPool.size(); i++) {
            constraintsPool.release(tmpSolverConstraintPool.getQuick(i));
        }
        tmpSolverConstraintPool.clear();

        for (int i = 0; i < tmpSolverFrictionConstraintPool.size(); i++) {
            constraintsPool.release(tmpSolverFrictionConstraintPool.getQuick(i));
        }
        tmpSolverFrictionConstraintPool.clear();

        return 0.0;
    }

    /**
     * Sequentially applies impulses.
     */
    @Override
    public double solveGroup(ObjectArrayList<CollisionObject> bodies, int numBodies, ObjectArrayList<PersistentManifold> manifoldPtr, int manifoldOffset, int numManifolds, ObjectArrayList<TypedConstraint> constraints, int constraintsOffset, int numConstraints, ContactSolverInfo infoGlobal, IDebugDraw debugDrawer, Dispatcher dispatcher) {
        BulletStats.pushProfile("solveGroup");
        try {
            // TODO: solver cache friendly
            if ((infoGlobal.solverMode & SolverMode.SOLVER_CACHE_FRIENDLY) != 0) {
                // you need to provide at least some bodies
                // SimpleDynamicsWorld needs to switch off SOLVER_CACHE_FRIENDLY
                assert (bodies != null);
                assert (numBodies != 0);
                return solveGroupCacheFriendly(manifoldPtr, manifoldOffset, numManifolds, constraints, constraintsOffset, numConstraints, infoGlobal /*,stackAlloc*/);
            }

            ContactSolverInfo info = new ContactSolverInfo(infoGlobal);

            int numIter = infoGlobal.numIterations;

            int totalPoints = 0;
            for (short j = 0; j < numManifolds; j++) {
                PersistentManifold manifold = manifoldPtr.getQuick(manifoldOffset + j);
                prepareConstraints(manifold, info);

                for (short p = 0; p < manifoldPtr.getQuick(manifoldOffset + j).getNumContacts(); p++) {
                    gOrder[totalPoints] = orderIndex(j, p);
                    totalPoints++;
                }
            }

            for (int j = 0; j < numConstraints; j++) {
                TypedConstraint constraint = constraints.getQuick(constraintsOffset + j);
                constraint.buildJacobian();
            }

            // should traverse the contacts random order...
            for (int iteration = 0; iteration < numIter; iteration++) {

                if ((infoGlobal.solverMode & SolverMode.SOLVER_RANDMIZE_ORDER) != 0) {
                    if ((iteration & 7) == 0) {
                        for (int j = 0; j < totalPoints; ++j) {
                            // JAVA NOTE: swaps references instead of copying values (but that's fine in this context)
                            long tmp = gOrder[j];
                            int swapi = randInt2(j + 1);
                            gOrder[j] = gOrder[swapi];
                            gOrder[swapi] = tmp;
                        }
                    }
                }

                for (int j = 0; j < numConstraints; j++) {
                    TypedConstraint constraint = constraints.getQuick(constraintsOffset + j);
                    constraint.solveConstraint(info.timeStep);
                }

                for (int j = 0; j < totalPoints; j++) {
                    PersistentManifold manifold = manifoldPtr.getQuick(manifoldOffset + orderManifoldIndex(gOrder[j]));
                    solve((RigidBody) manifold.getBody0(),
                            (RigidBody) manifold.getBody1(), manifold.getContactPoint(orderPointIndex(gOrder[j])), info);
                }

                for (int j = 0; j < totalPoints; j++) {
                    PersistentManifold manifold = manifoldPtr.getQuick(manifoldOffset + orderManifoldIndex(gOrder[j]));
                    solveFriction((RigidBody) manifold.getBody0(),
                            (RigidBody) manifold.getBody1(), manifold.getContactPoint(orderPointIndex(gOrder[j])), info);
                }
            }
            return 0.0;
        } finally {
            BulletStats.popProfile();
        }
    }

    protected void prepareConstraints(PersistentManifold manifoldPtr, ContactSolverInfo info) {
        RigidBody body0 = (RigidBody) manifoldPtr.getBody0();
        RigidBody body1 = (RigidBody) manifoldPtr.getBody1();

        // only necessary to refresh the manifold once (first iteration). The integration is done outside the loop

        //#ifdef FORCE_REFESH_CONTACT_MANIFOLDS
        //manifoldPtr->refreshContactPoints(body0->getCenterOfMassTransform(),body1->getCenterOfMassTransform());
        //#endif //FORCE_REFESH_CONTACT_MANIFOLDS
        int numpoints = manifoldPtr.getNumContacts();

        BulletStats.totalContactPoints += numpoints;

        Vector3d tmpVec = Stack.newVec();
        Vector3d tmpVec1 = Stack.newVec();
        Matrix3d tmpMat3 = Stack.newMat();

        Vector3d pos1 = Stack.newVec();
        Vector3d pos2 = Stack.newVec();
        Vector3d relPos1 = Stack.newVec();
        Vector3d relPos2 = Stack.newVec();
        Vector3d vel1 = Stack.newVec();
        Vector3d vel2 = Stack.newVec();
        Vector3d vel = Stack.newVec();
        Vector3d totalImpulse = Stack.newVec();
        Vector3d torqueAxis0 = Stack.newVec();
        Vector3d torqueAxis1 = Stack.newVec();
        Vector3d ftorqueAxis0 = Stack.newVec();
        Vector3d ftorqueAxis1 = Stack.newVec();

        Transform trans1 = Stack.newTrans();
        Transform trans2 = Stack.newTrans();

        for (int i = 0; i < numpoints; i++) {
            ManifoldPoint cp = manifoldPtr.getContactPoint(i);
            if (cp.getDistance() <= 0.0) {
                cp.getPositionWorldOnA(pos1);
                cp.getPositionWorldOnB(pos2);

                relPos1.sub(pos1, body0.getCenterOfMassPosition(tmpVec));
                relPos2.sub(pos2, body1.getCenterOfMassPosition(tmpVec));

                // this jacobian entry is re-used for all iterations
                Matrix3d mat1 = body0.getCenterOfMassTransform(trans1).basis;
                mat1.transpose();

                Matrix3d mat2 = body1.getCenterOfMassTransform(trans2).basis;
                mat2.transpose();

                JacobianEntry jac = jacobiansPool.get();
                jac.init(mat1, mat2,
                        relPos1, relPos2, cp.normalWorldOnB,
                        body0.getInvInertiaDiagLocal(tmpVec), body0.inverseMass,
                        body1.getInvInertiaDiagLocal(tmpVec1), body1.inverseMass);

                double jacDiagAB = jac.getDiagonal();
                jacobiansPool.release(jac);

                ConstraintPersistentData cpd = (ConstraintPersistentData) cp.userPersistentData;
                if (cpd != null) {
                    // might be invalid
                    cpd.persistentLifeTime++;
                    if (cpd.persistentLifeTime != cp.getLifeTime()) {
                        //printf("Invalid: cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd->m_persistentLifeTime,cp.getLifeTime());
                        //new (cpd) btConstraintPersistentData;
                        cpd.reset();
                        cpd.persistentLifeTime = cp.getLifeTime();

                    }// else printf("Persistent: cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd->m_persistentLifeTime,cp.getLifeTime());
                } else {
                    // todo: should this be in a pool?
                    //void* mem = btAlignedAlloc(sizeof(btConstraintPersistentData),16);
                    //cpd = new (mem)btConstraintPersistentData;
                    cpd = new ConstraintPersistentData();
                    //assert(cpd != null);

                    //printf("totalCpd = %i Created Ptr %x\n",totalCpd,cpd);
                    cp.userPersistentData = cpd;
                    cpd.persistentLifeTime = cp.getLifeTime();
                    //printf("CREATED: %x . cpd->m_persistentLifeTime = %i cp.getLifeTime() = %i\n",cpd,cpd->m_persistentLifeTime,cp.getLifeTime());
                }

                cpd.jacDiagABInv = 1.0 / jacDiagAB;

                // Dependent on Rigidbody A and B types, fetch the contact/friction response func
                // perhaps do a similar thing for friction/restutution combiner funcs...

                cpd.frictionSolverFunc = frictionDispatch[body0.frictionSolverType][body1.frictionSolverType];
                cpd.contactSolverFunc = contactDispatch[body0.contactSolverType][body1.contactSolverType];

                body0.getVelocityInLocalPoint(relPos1, vel1);
                body1.getVelocityInLocalPoint(relPos2, vel2);
                vel.sub(vel1, vel2);

                double rel_vel;
                rel_vel = cp.normalWorldOnB.dot(vel);

                double combinedRestitution = cp.combinedRestitution;

                cpd.penetration = cp.getDistance(); ///btScalar(info.m_numIterations);
                cpd.friction = cp.combinedFriction;
                cpd.restitution = restitutionCurve(rel_vel, combinedRestitution);
                if (cpd.restitution <= 0.0) {
                    cpd.restitution = 0.0;
                }

                // restitution and penetration work in same direction so
                // rel_vel

                double penVel = -cpd.penetration / info.timeStep;

                if (cpd.restitution > penVel) {
                    cpd.penetration = 0.0;
                }

                double relaxation = info.damping;
                if ((info.solverMode & SolverMode.SOLVER_USE_WARMSTARTING) != 0) {
                    cpd.appliedImpulse *= relaxation;
                } else {
                    cpd.appliedImpulse = 0.0;
                }

                // for friction
                cpd.prevAppliedImpulse = cpd.appliedImpulse;

                // re-calculate friction direction every frame, todo: check if this is really needed
                TransformUtil.planeSpace1(cp.normalWorldOnB, cpd.frictionWorldTangential0, cpd.frictionWorldTangential1);

                //#define NO_FRICTION_WARMSTART 1
                //#ifdef NO_FRICTION_WARMSTART
                cpd.accumulatedTangentImpulse0 = 0.0;
                cpd.accumulatedTangentImpulse1 = 0.0;
                //#endif //NO_FRICTION_WARMSTART
                double denom0 = body0.computeImpulseDenominator(pos1, cpd.frictionWorldTangential0);
                double denom1 = body1.computeImpulseDenominator(pos2, cpd.frictionWorldTangential0);
                double denom = relaxation / (denom0 + denom1);
                cpd.jacDiagABInvTangent0 = denom;

                denom0 = body0.computeImpulseDenominator(pos1, cpd.frictionWorldTangential1);
                denom1 = body1.computeImpulseDenominator(pos2, cpd.frictionWorldTangential1);
                denom = relaxation / (denom0 + denom1);
                cpd.jacDiagABInvTangent1 = denom;

                totalImpulse.scale(cpd.appliedImpulse, cp.normalWorldOnB);


                torqueAxis0.cross(relPos1, cp.normalWorldOnB);
                cpd.angularComponentA.set(torqueAxis0);
                body0.getInvInertiaTensorWorld(tmpMat3).transform(cpd.angularComponentA);

                torqueAxis1.cross(relPos2, cp.normalWorldOnB);
                cpd.angularComponentB.set(torqueAxis1);
                body1.getInvInertiaTensorWorld(tmpMat3).transform(cpd.angularComponentB);


                ftorqueAxis0.cross(relPos1, cpd.frictionWorldTangential0);
                cpd.frictionAngularComponent0A.set(ftorqueAxis0);
                body0.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent0A);


                ftorqueAxis1.cross(relPos1, cpd.frictionWorldTangential1);
                cpd.frictionAngularComponent1A.set(ftorqueAxis1);
                body0.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent1A);


                ftorqueAxis0.cross(relPos2, cpd.frictionWorldTangential0);
                cpd.frictionAngularComponent0B.set(ftorqueAxis0);
                body1.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent0B);


                ftorqueAxis1.cross(relPos2, cpd.frictionWorldTangential1);
                cpd.frictionAngularComponent1B.set(ftorqueAxis1);
                body1.getInvInertiaTensorWorld(tmpMat3).transform(cpd.frictionAngularComponent1B);

                ///

                // apply previous frames impulse on both bodies
                body0.applyImpulse(totalImpulse, relPos1);

                tmpVec.negate(totalImpulse);
                body1.applyImpulse(tmpVec, relPos2);
            }
        }

        Stack.subVec(14);
        Stack.subTrans(2);
        Stack.subMat(1);
    }

    @SuppressWarnings("unused")
    public double solveCombinedContactFriction(RigidBody body0, RigidBody body1, ManifoldPoint cp, ContactSolverInfo info) {
        double maxImpulse = 0.0;
        if (cp.getDistance() <= 0.0) {
            double impulse = ContactConstraint.resolveSingleCollisionCombined(body0, body1, cp, info);
            if (maxImpulse < impulse) {
                maxImpulse = impulse;
            }
        }
        return maxImpulse;
    }

    @SuppressWarnings("UnusedReturnValue")
    protected double solve(RigidBody body0, RigidBody body1, ManifoldPoint cp, ContactSolverInfo info) {
        double maxImpulse = 0.0;
        if (cp.getDistance() <= 0.0) {
            ConstraintPersistentData cpd = (ConstraintPersistentData) cp.userPersistentData;
            double impulse = cpd.contactSolverFunc.resolveContact(body0, body1, cp, info);
            if (maxImpulse < impulse) {
                maxImpulse = impulse;
            }
        }
        return maxImpulse;
    }

    @SuppressWarnings("UnusedReturnValue")
    protected double solveFriction(RigidBody body0, RigidBody body1, ManifoldPoint cp, ContactSolverInfo info) {
        if (cp.getDistance() <= 0.0) {
            ConstraintPersistentData cpd = (ConstraintPersistentData) cp.userPersistentData;
            cpd.frictionSolverFunc.resolveContact(body0, body1, cp, info);
        }
        return 0.0;
    }

    @Override
    public void reset() {
        btSeed2 = 0;
    }

    /**
     * Advanced: Override the default contact solving function for contacts, for certain types of rigidbody<br>
     * See RigidBody.contactSolverType and RigidBody.frictionSolverType
     */
    @SuppressWarnings("unused")
    public void setContactSolverFunc(ContactSolverFunc func, int type0, int type1) {
        contactDispatch[type0][type1] = func;
    }

    /**
     * Advanced: Override the default friction solving function for contacts, for certain types of rigidbody<br>
     * See RigidBody.contactSolverType and RigidBody.frictionSolverType
     */
    @SuppressWarnings("unused")
    public void setFrictionSolverFunc(ContactSolverFunc func, int type0, int type1) {
        frictionDispatch[type0][type1] = func;
    }

    @SuppressWarnings("unused")
    public void setRandSeed(long seed) {
        btSeed2 = seed;
    }

    @SuppressWarnings("unused")
    public long getRandSeed() {
        return btSeed2;
    }

    /// /////////////////////////////////////////////////////////////////////////

    public static int orderManifoldIndex(long v) {
        return unpackHigh(v);
    }

    public static int orderPointIndex(long v) {
        return unpackLow(v);
    }

    public static long orderIndex(int manifoldIndex, int pointIndex) {
        return pack(manifoldIndex, pointIndex);
    }

}
