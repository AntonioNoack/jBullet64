/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.dynamics.constraintsolver;

import javax.vecmath.Vector3d;

/**
 * 1D constraint along a normal axis between bodyA and bodyB. It can be combined
 * to solve contact and friction constraints.
 *
 * @author jezek2
 */
public class SolverConstraint {

    public final Vector3d relpos1CrossNormal = new Vector3d();
    public final Vector3d contactNormal = new Vector3d();

    public final Vector3d relpos2CrossNormal = new Vector3d();
    public final Vector3d angularComponentA = new Vector3d();

    public final Vector3d angularComponentB = new Vector3d();

    public double appliedPushImpulse;

    public double appliedImpulse;
    public int solverBodyIdA;
    public int solverBodyIdB;

    public double friction;
    public double restitution;
    public double jacDiagABInv;
    public double penetration;

    public SolverConstraintType constraintType;
    public int frictionIndex;
    public Object originalContactPoint;

}
