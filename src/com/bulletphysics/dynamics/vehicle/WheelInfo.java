package com.bulletphysics.dynamics.vehicle;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * WheelInfo contains information per wheel about friction and suspension.
 *
 * @author jezek2
 */
public class WheelInfo {

    public final RaycastInfo raycastInfo = new RaycastInfo();

    public final Transform worldTransform = new Transform();

    public final Vector3d chassisConnectionPointCS = new Vector3d(); // const
    public final Vector3d wheelDirectionCS = new Vector3d(); // const
    public final Vector3d wheelAxleCS = new Vector3d(); // const or modified by steering
    public double suspensionRestLength1; // const
    public double maxSuspensionTravelCm;
    public double wheelsRadius; // const
    public double suspensionStiffness; // const
    public double wheelsDampingCompression; // const
    public double wheelsDampingRelaxation; // const
    public double frictionSlip;
    public double steering;
    public double rotation;
    public double deltaRotation;
    public double rollInfluence;

    public double engineForce;

    public double brake;

    public boolean bIsFrontWheel;

    public Object clientInfo; // can be used to store pointer to sync transforms...

    public double clippedInvContactDotSuspension;
    public double suspensionRelativeVelocity;
    // calculated by suspension
    public double wheelsSuspensionForce;
    public double skidInfo;

    public WheelInfo(WheelInfoConstructionInfo ci) {
        suspensionRestLength1 = ci.suspensionRestLength;
        maxSuspensionTravelCm = ci.maxSuspensionTravelCm;

        wheelsRadius = ci.wheelRadius;
        suspensionStiffness = ci.suspensionStiffness;
        wheelsDampingCompression = ci.wheelsDampingCompression;
        wheelsDampingRelaxation = ci.wheelsDampingRelaxation;
        chassisConnectionPointCS.set(ci.chassisConnectionCS);
        wheelDirectionCS.set(ci.wheelDirectionCS);
        wheelAxleCS.set(ci.wheelAxleCS);
        frictionSlip = ci.frictionSlip;
        steering = 0;
        engineForce = 0;
        rotation = 0;
        deltaRotation = 0;
        brake = 0;
        rollInfluence = 0.1;
        bIsFrontWheel = ci.bIsFrontWheel;
    }

    public double getSuspensionRestLength() {
        return suspensionRestLength1;
    }

    public void updateWheel(RigidBody chassis, RaycastInfo raycastInfo) {
        if (raycastInfo.isInContact) {
            double project = raycastInfo.contactNormalWS.dot(raycastInfo.wheelDirectionWS);
            Vector3d chassis_velocity_at_contactPoint = Stack.newVec();
            Vector3d relPos = Stack.newVec();
            relPos.sub(raycastInfo.contactPointWS, chassis.getCenterOfMassPosition(Stack.newVec()));
            chassis.getVelocityInLocalPoint(relPos, chassis_velocity_at_contactPoint);
            double projVel = raycastInfo.contactNormalWS.dot(chassis_velocity_at_contactPoint);
            if (project >= -0.1) {
                suspensionRelativeVelocity = 0;
                clippedInvContactDotSuspension = 1 / 0.1;
            } else {
                double inv = -1 / project;
                suspensionRelativeVelocity = projVel * inv;
                clippedInvContactDotSuspension = inv;
            }
        } else {
            // Not in contact : position wheel in a nice (rest length) position
            raycastInfo.suspensionLength = getSuspensionRestLength();
            suspensionRelativeVelocity = 0;
            raycastInfo.contactNormalWS.negate(raycastInfo.wheelDirectionWS);
            clippedInvContactDotSuspension = 1;
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    public static class RaycastInfo {
        // set by raycaster
        public final Vector3d contactNormalWS = new Vector3d(); // contact normal
        public final Vector3d contactPointWS = new Vector3d(); // raycast hitpoint
        public double suspensionLength;
        public final Vector3d hardPointWS = new Vector3d(); // raycast starting point
        public final Vector3d wheelDirectionWS = new Vector3d(); // direction in worldspace
        public final Vector3d wheelAxleWS = new Vector3d(); // axle in worldspace
        public boolean isInContact;
        public Object groundObject; // could be general void* ptr
    }

}
