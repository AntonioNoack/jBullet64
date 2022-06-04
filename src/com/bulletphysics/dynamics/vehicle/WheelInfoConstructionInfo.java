package com.bulletphysics.dynamics.vehicle;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class WheelInfoConstructionInfo {

    public final Vector3d chassisConnectionCS = new Vector3d();
    public final Vector3d wheelDirectionCS = new Vector3d();
    public final Vector3d wheelAxleCS = new Vector3d();
    public double suspensionRestLength;
    public double maxSuspensionTravelCm;
    public double wheelRadius;

    public double suspensionStiffness;
    public double wheelsDampingCompression;
    public double wheelsDampingRelaxation;
    public double frictionSlip;
    public boolean bIsFrontWheel;

}
