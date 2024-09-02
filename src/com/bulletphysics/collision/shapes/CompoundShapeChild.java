package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;

/**
 * Compound shape child.
 *
 * @author jezek2
 */
public class CompoundShapeChild {

    public final Transform transform = new Transform();
    public CollisionShape childShape;
    public BroadphaseNativeType childShapeType;
    public double childMargin;

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof CompoundShapeChild)) return false;
        CompoundShapeChild child = (CompoundShapeChild) obj;
        return transform.equals(child.transform) &&
                childShape == child.childShape &&
                childShapeType == child.childShapeType &&
                childMargin == child.childMargin;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 19 * hash + transform.hashCode();
        hash = 19 * hash + childShape.hashCode();
        hash = 19 * hash + childShapeType.hashCode();
        hash = 19 * hash + Long.hashCode(Double.doubleToLongBits(childMargin));
        return hash;
    }

}
