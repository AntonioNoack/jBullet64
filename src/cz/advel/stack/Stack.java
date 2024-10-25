package cz.advel.stack;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.nio.BufferUnderflowException;
import java.util.HashMap;

/**
 * this class is now fully thread safe :)
 * is is used to quickly borrow instances from specific classes used in bullet (javax.vecmath)
 */
public class Stack {

    private int vectorPosition = 0, matrixPosition = 0, quatPosition = 0, transPosition = 0;
    private int manifoldPosition = 0, doublePtrPosition = 0;

    public static int limit = 65536;

    private Vector3d[] vectors = new Vector3d[32];
    private Matrix3d[] matrices = new Matrix3d[32];
    private Quat4d[] quads = new Quat4d[32];
    private Transform[] transforms = new Transform[32];
    private ManifoldPoint[] manifoldPoints = new ManifoldPoint[32];
    private double[][] doublePtrs = new double[32][];

    private static final ThreadLocal<Stack> instances = ThreadLocal.withInitial(Stack::new);

    // I either didn't find the library, or it was too large for my liking:
    // I rewrote the main functionalities

    public void reset2(boolean printSlack) {
        if (printSlack) {
            System.out.println("[BulletStack]: Slack: " +
                    vectorPosition + " vectors, " +
                    matrixPosition + " matrices, " +
                    quatPosition + " quaternions, " +
                    transPosition + " transforms, " +
                    manifoldPosition + " manifolds, " +
                    doublePtrPosition + " doublePtrs"
            );
        }
        vectorPosition = 0;
        matrixPosition = 0;
        quatPosition = 0;
        transPosition = 0;
        manifoldPosition = 0;
        doublePtrPosition = 0;
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

    int depth = 0;

    public static int[] getPosition(int[] dst) {
        if (dst == null) return getPosition(new int[6]);
        Stack instance = instances.get();
        dst[0] = instance.vectorPosition;
        dst[1] = instance.matrixPosition;
        dst[2] = instance.quatPosition;
        dst[3] = instance.transPosition;
        dst[4] = instance.manifoldPosition;
        dst[5] = instance.doublePtrPosition;
        // System.out.println("Getting state [" + instance.depth + "] at " + Arrays.toString(dst));
        instance.depth++;
        return dst;
    }

    public static void reset(int[] positions) {
        Stack instance = instances.get();
        instance.vectorPosition = positions[0];
        instance.matrixPosition = positions[1];
        instance.quatPosition = positions[2];
        instance.transPosition = positions[3];
        instance.manifoldPosition = positions[4];
        instance.doublePtrPosition = positions[5];
        instance.depth--;
    }

    private static void checkUnderflow(Stack stack) {
        if (stack.vectorPosition < 0) throw new BufferUnderflowException();
    }

    public static void subVec(int delta) {
        Stack stack = instances.get();
        stack.vectorPosition -= delta;
        printCaller("subVec(" + delta + ")", 2, stack.vectorPosition);
        checkUnderflow(stack);
    }

    public static void subMat(int delta) {
        Stack stack = instances.get();
        stack.matrixPosition -= delta;
        checkUnderflow(stack);
    }

    public static void subQuat(int delta) {
        Stack stack = instances.get();
        stack.quatPosition -= delta;
        checkUnderflow(stack);
    }

    public static void subTrans(int delta) {
        Stack stack = instances.get();
        stack.transPosition -= delta;
        checkUnderflow(stack);
    }

    public static void resetVec(int position) {
        Stack stack = instances.get();
        stack.vectorPosition = position;
    }

    public static void resetMat(int position) {
        Stack stack = instances.get();
        stack.matrixPosition = position;
    }

    public static void resetTrans(int position) {
        Stack stack = instances.get();
        stack.transPosition = position;
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
        for (int i = 0, l = doublePtrs.length; i < l; i++) {
            doublePtrs[i] = new double[4];
        }
    }

    public HashMap<String, Integer> usageAnalyser;

    public void printClassUsage2() {
    }

    public static void printClassUsage() {
    }

    private void checkLeaking(int newSize) {
        if (newSize > limit) throw new OutOfMemoryError("Reached stack limit " + limit + ", probably leaking");
    }

    public Vector3d newVec2() {
        Vector3d[] vts = vectors;
        if (vectorPosition >= vts.length) {
            int newSize = vts.length * 2;
            checkLeaking(newSize);
            Vector3d[] values = new Vector3d[newSize];
            System.arraycopy(vts, 0, values, 0, vts.length);
            for (int i = vts.length; i < newSize; i++) {
                values[i] = new Vector3d();
            }
            vectors = vts = values;
        }
        return vts[vectorPosition++];
    }

    public static boolean shallPrintCallers = false;

    private static void printCaller(String type, int depth, int pos) {
        if (shallPrintCallers) {
            StackTraceElement[] elements = new Throwable().getStackTrace();
            if (elements != null && depth < elements.length) {
                StringBuilder builder = new StringBuilder();
                for (int i = 0; i < elements.length; i++) {
                    builder.append("  ");
                }
                builder.append(type).append(" on ").append(elements[depth]);
                builder.append(" (").append(pos).append(")");
                System.out.println(builder);
            }
        }
    }

    public static Vector3d newVec() {
        Stack stack = instances.get();
        printCaller("newVec()", 2, stack.vectorPosition);
        Vector3d v = stack.newVec2();
        v.set(0.0, 0.0, 0.0);
        return v;
    }

    public static Vector3d newVec(double xyz) {
        Stack stack = instances.get();
        printCaller("newVec(d)", 2, stack.vectorPosition);
        Vector3d v = stack.newVec2();
        v.set(xyz, xyz, xyz);
        return v;
    }

    public static Vector3d newVec(double x, double y, double z) {
        Stack stack = instances.get();
        printCaller("newVec(xyz)", 2, stack.vectorPosition);
        Vector3d v = stack.newVec2();
        v.set(x, y, z);
        return v;
    }

    public static Vector3d newVec(Vector3d src) {
        Stack stack = instances.get();
        printCaller("newVec(src)", 2, stack.vectorPosition);
        Vector3d value = stack.newVec2();
        value.set(src);
        return value;
    }

    public Quat4d newQuat2() {
        Quat4d[] qds = quads;
        if (quatPosition >= qds.length) {
            int newSize = qds.length * 2;
            checkLeaking(newSize);
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
        Quat4d v = instances.get().newQuat2();
        v.set(0.0, 0.0, 0.0, 1.0);
        return v;
    }

    public static Quat4d newQuat(Quat4d base) {
        Quat4d v = instances.get().newQuat2();
        v.set(base);
        return v;
    }

    public static Matrix3d newMat(Matrix3d base) {
        Matrix3d v = instances.get().newMat2();
        v.set(base);
        return v;
    }

    public Matrix3d newMat2() {
        Matrix3d[] mts = matrices;
        if (matrixPosition >= mts.length) {
            int newSize = mts.length * 2;
            checkLeaking(newSize);
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
        Transform[] trs = transforms;
        if (transPosition >= trs.length) {
            int newSize = trs.length * 2;
            checkLeaking(newSize);
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
        ManifoldPoint[] pts = manifoldPoints;
        if (manifoldPosition >= pts.length) {
            int newSize = pts.length * 2;
            // checkLeaking(newSize);
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

    public double[] newDoublePtr2() {
        double[][] pts = doublePtrs;
        if (doublePtrPosition >= pts.length) {
            int newSize = pts.length * 2;
            // checkLeaking(newSize);
            double[][] values = new double[newSize][];
            System.arraycopy(pts, 0, values, 0, pts.length);
            for (int i = pts.length; i < newSize; i++) {
                values[i] = new double[4];
            }
            doublePtrs = pts = values;
        }
        return pts[doublePtrPosition++];
    }

    public static double[] newDoublePtr() {
        return instances.get().newDoublePtr2();
    }

    public static void subDoublePtr(int delta) {
        Stack stack = instances.get();
        stack.quatPosition -= delta;
        checkUnderflow(stack);
    }

    public static double[] borrowDoublePtr() {
        Stack stack = instances.get();
        printCaller("borrowDoublePtr()", 2, stack.doublePtrPosition);
        double[] v = stack.newDoublePtr2();
        stack.doublePtrPosition--;
        return v;
    }

    public static Vector3d borrowVec() {
        Stack stack = instances.get();
        printCaller("borrowVec()", 2, stack.vectorPosition);
        Vector3d v = stack.newVec2();
        stack.vectorPosition--;
        return v;
    }

    public static Vector3d borrowVec(Vector3d src) {
        Stack stack = instances.get();
        printCaller("borrowVec(src)", 2, stack.vectorPosition);
        Vector3d v = stack.newVec2();
        stack.vectorPosition--;
        v.set(src);
        return v;
    }

    public static Quat4d borrowQuat() {
        Stack stack = instances.get();
        Quat4d v = stack.newQuat2();
        stack.quatPosition--;
        return v;
    }

    public static Matrix3d borrowMat() {
        Stack stack = instances.get();
        Matrix3d m = stack.newMat2();
        stack.matrixPosition--;
        return m;
    }

    public static Matrix3d borrowMat(Matrix3d set) {
        Stack stack = instances.get();
        Matrix3d m = stack.newMat2();
        m.set(set);
        stack.matrixPosition--;
        return m;
    }

    public static Transform borrowTrans() {
        Stack stack = instances.get();
        Transform t = stack.newTrans2();
        stack.transPosition--;
        return t;
    }

    public static void libraryCleanCurrentThread() {

    }

}
