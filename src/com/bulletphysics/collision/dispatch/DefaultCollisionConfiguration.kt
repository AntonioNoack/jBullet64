package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.narrowphase.ConvexPenetrationDepthSolver;
import com.bulletphysics.collision.narrowphase.GjkEpaPenetrationDepthSolver;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;
import com.bulletphysics.extras.gimpact.GImpactCollisionAlgorithm;

import static com.bulletphysics.collision.broadphase.BroadphaseNativeType.SPHERE_SHAPE_PROXYTYPE;
import static com.bulletphysics.collision.broadphase.BroadphaseNativeType.STATIC_PLANE_PROXYTYPE;

/**
 * Default implementation of {@link CollisionConfiguration}. Provides all core
 * collision algorithms. Some extra algorithms (like {@link GImpactCollisionAlgorithm GImpact})
 * must be registered manually by calling appropriate register method.
 *
 * @author jezek2
 */
public class DefaultCollisionConfiguration implements CollisionConfiguration {

    //default simplex/penetration depth solvers
    protected ConvexPenetrationDepthSolver pdSolver;

    //default CreationFunctions, filling the m_doubleDispatch table
    protected CollisionAlgorithmCreateFunc convexConvexCreateFunc;
    protected CollisionAlgorithmCreateFunc convexConcaveCreateFunc;
    protected CollisionAlgorithmCreateFunc swappedConvexConcaveCreateFunc;
    protected CollisionAlgorithmCreateFunc compoundCreateFunc;
    protected CollisionAlgorithmCreateFunc swappedCompoundCreateFunc;
    protected CollisionAlgorithmCreateFunc emptyCreateFunc;
    protected CollisionAlgorithmCreateFunc sphereSphereCF;
    protected CollisionAlgorithmCreateFunc planeConvexCF;
    protected CollisionAlgorithmCreateFunc convexPlaneCF;

    public DefaultCollisionConfiguration() {

        pdSolver = new GjkEpaPenetrationDepthSolver();

		/*
		//default CreationFunctions, filling the m_doubleDispatch table
		*/
        convexConvexCreateFunc = new ConvexConvexAlgorithm.CreateFunc(new VoronoiSimplexSolver(), pdSolver);
        convexConcaveCreateFunc = new ConvexConcaveCollisionAlgorithm.CreateFunc();
        swappedConvexConcaveCreateFunc = new ConvexConcaveCollisionAlgorithm.SwappedCreateFunc();
        compoundCreateFunc = new CompoundCollisionAlgorithm.CreateFunc();
        swappedCompoundCreateFunc = new CompoundCollisionAlgorithm.SwappedCreateFunc();
        emptyCreateFunc = new EmptyAlgorithm.CreateFunc();

        sphereSphereCF = new SphereSphereCollisionAlgorithm.CreateFunc();

        // convex versus plane
        convexPlaneCF = new ConvexPlaneCollisionAlgorithm.CreateFunc();
        planeConvexCF = new ConvexPlaneCollisionAlgorithm.CreateFunc();
        planeConvexCF.swapped = true;
    }

    @Override
    public CollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc(BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1) {
        if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE)) {
            return sphereSphereCF;
        }

        if (proxyType0.isConvex() && (proxyType1 == STATIC_PLANE_PROXYTYPE)) {
            return convexPlaneCF;
        }

        if (proxyType1.isConvex() && (proxyType0 == STATIC_PLANE_PROXYTYPE)) {
            return planeConvexCF;
        }

        if (proxyType0.isConvex() && proxyType1.isConvex()) {
            return convexConvexCreateFunc;
        }

        if (proxyType0.isConvex() && proxyType1.isConcave()) {
            return convexConcaveCreateFunc;
        }

        if (proxyType1.isConvex() && proxyType0.isConcave()) {
            return swappedConvexConcaveCreateFunc;
        }

        if (proxyType0.isCompound()) {
            return compoundCreateFunc;
        } else if (proxyType1.isCompound()) {
            return swappedCompoundCreateFunc;
        }

        // failed to find an algorithm
        return emptyCreateFunc;
    }

}
