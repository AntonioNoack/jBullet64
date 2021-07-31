package cz.advel.stack;

import com.bulletphysics.collision.broadphase.DbvtAabbMm;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.HashMap;
import java.util.Map;

/**
 * this class is now fully thread safe :)
 * is is used to quickly borrow instances from specific classes used in bullet (javax.vecmath)
 */
public class Stack {

    private int vectorPosition = 0, matrixPosition = 0, quatPosition = 0, transPosition = 0;
    private int manifoldPosition = 0;

    private Vector3d[] vectors = new Vector3d[32];
    private Matrix3d[] matrices = new Matrix3d[32];
    private Quat4d[] quads = new Quat4d[32];
    private Transform[] transforms = new Transform[32];
    private ManifoldPoint[] manifoldPoints = new ManifoldPoint[32];

    private static final ThreadLocal<Stack> instances = ThreadLocal.withInitial(Stack::new);

    // I either didn't find the library, or it was too large for my liking:
    // I rewrote the main functionalities

    public void reset2(boolean printSlack) {
        if (printSlack) {
            System.out.println("Stack-Slack: " +
                    vectorPosition + " vectors, " +
                    matrixPosition + " matrices, " +
                    quatPosition + " quaternions, " +
                    transPosition + " transforms, " +
                    manifoldPosition + " manifolds"
            );
        }
        vectorPosition = 0;
        matrixPosition = 0;
        quatPosition = 0;
        transPosition = 0;
        manifoldPosition = 0;
    }

    public static void reset(boolean printSlack) {
        instances.get().reset2(printSlack);
    }

    public void reset2(int vec, int mat, int quat, int trans) {
        vectorPosition = vec;
        matrixPosition = mat;
        quatPosition = quat;
        transPosition = trans;
    }

    public static void reset(int vec, int mat, int quat, int trans) {
        instances.get().reset2(vec, mat, quat, trans);
    }

    // public static Vector4d borrowVector4d;
    public static DbvtAabbMm borrowAABBMM = new DbvtAabbMm();

    public static int getVecPosition() {
        return instances.get().vectorPosition;
    }

    public static int getMatPosition() {
        return instances.get().matrixPosition;
    }

    public static int getQuatPosition() {
        return instances.get().quatPosition;
    }

    public static int getTransPosition() {
        return instances.get().transPosition;
    }

    public static void subVec(int delta) {
        Stack stack = instances.get();
        stack.onAllocation("vec", -1, -delta);
        stack.vectorPosition -= delta;
    }

    public static void subMat(int delta) {
        Stack stack = instances.get();
        stack.onAllocation("mat", -1, -delta);
        stack.matrixPosition -= delta;
    }

    public static void subQuat(int delta) {
        Stack stack = instances.get();
        stack.onAllocation("quat", -1, -delta);
        stack.quatPosition -= delta;
    }

    public static void subTrans(int delta) {
        Stack stack = instances.get();
        stack.onAllocation("trans", -1, -delta);
        stack.transPosition -= delta;
    }

    public static void resetVec(int position) {
        Stack stack = instances.get();
        stack.onAllocation("vec", -1, position - stack.vectorPosition);
        stack.vectorPosition = position;
    }

    public static void resetMat(int position) {
        Stack stack = instances.get();
        stack.onAllocation("mat", -1, position - stack.matrixPosition);
        instances.get().matrixPosition = position;
    }

    public static void resetTrans(int position) {
        Stack stack = instances.get();
        stack.onAllocation("trans", -1, position - stack.transPosition);
        instances.get().transPosition = position;
    }

    public void printSizes2() {
        System.out.println("[BulletStack]: " +
                vectors.length + " vectors, " +
                matrices.length + " matrices, " +
                quads.length + " quads, " +
                transforms.length + " transforms, " +
                manifoldPoints.length + " manifold points");
    }

    public static void printSizes() {
        instances.get().printSizes2();
    }

    {
        for (int i = 0, l = vectors.length; i < l; i++) {
            vectors[i] = new Vector3d();
        }
        for (int i = 0, l = matrices.length; i < l; i++) {
            matrices[i] = new Matrix3d();
        }
        for (int i = 0, l = quads.length; i < l; i++) {
            quads[i] = new Quat4d();
        }
        for (int i = 0, l = transforms.length; i < l; i++) {
            transforms[i] = new Transform();
        }
        for (int i = 0, l = manifoldPoints.length; i < l; i++) {
            manifoldPoints[i] = new ManifoldPoint();
        }
    }

    public HashMap<String, Integer> usageAnalyser;

    public void printClassUsage2() {
        if (usageAnalyser != null) {
            usageAnalyser.entrySet().stream().sorted(Map.Entry.comparingByValue()).forEach((entry) -> {
                System.out.println(entry.getValue() + "x " + entry.getKey());
            });
        }
        usageAnalyser = new HashMap<>();
    }

    public static void printClassUsage() {
        instances.get().printClassUsage2();
    }

    private void onAllocation(String prefix, int depthExtra, int multiplier) {
        if (usageAnalyser == null) return;
        StackTraceElement[] trace = new Throwable().getStackTrace();
        int indexOfInterest = 3 + depthExtra;
        if (trace.length > indexOfInterest) {
            StackTraceElement interest = trace[indexOfInterest];
            String name = prefix + ":" + interest.getClassName() + "." + interest.getMethodName();
            int old = 0;
            Integer old2 = usageAnalyser.get(name);
            if (old2 != null) old = old2;
            usageAnalyser.put(name, old + multiplier);
        }
    }

    public Vector3d newVec2() {
        if (usageAnalyser != null) onAllocation("vec", 0, 1);
        Vector3d[] vts = vectors;
        if (vectorPosition >= vts.length) {
            int newSize = vts.length * 2;
            Vector3d[] values = new Vector3d[newSize];
            System.arraycopy(vts, 0, values, 0, vts.length);
            for (int i = vts.length; i < newSize; i++) {
                values[i] = new Vector3d();
            }
            vectors = vts = values;
        }
        return vts[vectorPosition++];
    }

    public static Vector3d newVec() {
        return instances.get().newVec2();
    }

    public static Vector3d newVec(Vector3d src) {
        Vector3d value = instances.get().newVec2();
        value.set(src);
        return value;
    }

    public Quat4d newQuat2() {
        if (usageAnalyser != null) onAllocation("quat", 0, 1);
        Quat4d[] qds = quads;
        if (quatPosition >= qds.length) {
            int newSize = qds.length * 2;
            Quat4d[] values = new Quat4d[newSize];
            System.arraycopy(qds, 0, values, 0, qds.length);
            for (int i = qds.length; i < newSize; i++) {
                values[i] = new Quat4d();
            }
            quads = qds = values;
        }
        return qds[quatPosition++];
    }

    public static Quat4d newQuat() {
        return instances.get().newQuat2();
    }

    public static Matrix3d newMat(Matrix3d base) {
        Matrix3d v = instances.get().newMat2();
        v.set(base);
        return v;
    }

    public Matrix3d newMat2() {
        if (usageAnalyser != null) onAllocation("mat", 0, 1);
        Matrix3d[] mts = matrices;
        if (matrixPosition >= mts.length) {
            int newSize = mts.length * 2;
            Matrix3d[] values = new Matrix3d[newSize];
            System.arraycopy(mts, 0, values, 0, mts.length);
            for (int i = mts.length; i < newSize; i++) {
                values[i] = new Matrix3d();
            }
            matrices = mts = values;
        }
        Matrix3d m = mts[matrixPosition++];
        if (m == null) throw new RuntimeException();
        return m;
    }

    public static Matrix3d newMat() {
        return instances.get().newMat2();
    }

    public Transform newTrans2() {
        if (usageAnalyser != null) onAllocation("trans", 0, 1);
        Transform[] trs = transforms;
        if (transPosition >= trs.length) {
            int newSize = trs.length * 2;
            Transform[] values = new Transform[newSize];
            System.arraycopy(trs, 0, values, 0, trs.length);
            for (int i = trs.length; i < newSize; i++) {
                values[i] = new Transform();
            }
            transforms = trs = values;
        }
        Transform m = trs[transPosition++];
        if (m == null) throw new RuntimeException();
        return m;
    }

    public static Transform newTrans() {
        return instances.get().newTrans2();
    }

    public static Transform newTrans(Transform base) {
        Transform v = instances.get().newTrans2();
        v.set(base);
        return v;
    }

    public ManifoldPoint newManifoldPoint2() {
        if (usageAnalyser != null) onAllocation("manifold", 0, 1);
        ManifoldPoint[] pts = manifoldPoints;
        if (manifoldPosition >= pts.length) {
            int newSize = pts.length * 2;
            ManifoldPoint[] values = new ManifoldPoint[newSize];
            System.arraycopy(pts, 0, values, 0, pts.length);
            for (int i = pts.length; i < newSize; i++) {
                values[i] = new ManifoldPoint();
            }
            manifoldPoints = pts = values;
        }
        return pts[manifoldPosition++];
    }

    public static ManifoldPoint newManifoldPoint() {
        return instances.get().newManifoldPoint2();
    }

    public static Vector3d borrowVec() {
        Stack stack = instances.get();
        Vector3d v = stack.newVec2();
        if (stack.usageAnalyser != null) stack.onAllocation("vec", -1, -1);
        stack.vectorPosition--;
        return v;
    }

    public static Vector3d borrowVec(Vector3d set) {
        Stack stack = instances.get();
        Vector3d v = stack.newVec2();
        if (stack.usageAnalyser != null) stack.onAllocation("vec", -1, -1);
        stack.vectorPosition--;
        v.set(set);
        return v;
    }

    public static Quat4d borrowQuat() {
        Stack stack = instances.get();
        Quat4d v = stack.newQuat2();
        if (stack.usageAnalyser != null) stack.onAllocation("quat", -1, -1);
        stack.quatPosition--;
        return v;
    }

    public static Matrix3d borrowMat() {
        Stack stack = instances.get();
        Matrix3d m = stack.newMat2();
        if (stack.usageAnalyser != null) stack.onAllocation("mat", -1, -1);
        stack.matrixPosition--;
        return m;
    }

    public static Matrix3d borrowMat(Matrix3d set) {
        Stack stack = instances.get();
        Matrix3d m = stack.newMat2();
        m.set(set);
        if (stack.usageAnalyser != null) stack.onAllocation("mat", -1, -1);
        stack.matrixPosition--;
        return m;
    }

    public static Transform borrowTrans() {
        Stack stack = instances.get();
        Transform t = stack.newTrans2();
        if (stack.usageAnalyser != null) stack.onAllocation("trans", -1, -1);
        stack.transPosition--;
        return t;
    }

    public static void libraryCleanCurrentThread() {

    }

}
