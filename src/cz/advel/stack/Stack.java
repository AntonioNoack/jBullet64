package cz.advel.stack;

import com.bulletphysics.collision.broadphase.DbvtAabbMm;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.lang.reflect.InvocationTargetException;

public class Stack {

    // I either didn't find the library, or it was too large for my liking:
    // I rewrote the main functionalities

    public static void reset() {
        vectorPosition = 0;
        matrixPosition = 0;
        quatPosition = 0;
        transformPosition = 0;
        manifoldPosition = 0;
    }

    public static void reset(int vec, int mat, int quat, int trans) {
        vectorPosition = vec;
        matrixPosition = mat;
        quatPosition = quat;
        transformPosition = trans;
    }

    // public static Vector4d borrowVector4d;
    public static DbvtAabbMm borrowAABBMM = new DbvtAabbMm();

    public static int getVecPosition() {
        return vectorPosition;
    }

    public static int getMatPosition() {
        return matrixPosition;
    }

    public static int getQuatPosition() {
        return quatPosition;
    }

    public static int getTransPosition() {
        return transformPosition;
    }

    public static void subVec(int delta) {
        vectorPosition -= delta;
    }

    public static void subMat(int delta) {
        matrixPosition -= delta;
    }

    public static void subQuat(int delta) {
        quatPosition -= delta;
    }

    public static void resetVec(int position) {
        vectorPosition = position;
    }

    public static void resetMat(int position) {
        matrixPosition = position;
    }

    private static int vectorPosition = 0, matrixPosition = 0, quatPosition = 0, transformPosition = 0;
    private static int manifoldPosition = 0;

    private static Vector3d[] vectors = new Vector3d[128];
    private static Matrix3d[] matrices = new Matrix3d[128];
    private static Quat4d[] quads = new Quat4d[128];
    private static Transform[] transforms = new Transform[128];
    private static ManifoldPoint[] manifoldPoints = new ManifoldPoint[128];

    public static void printSizes() {
        System.out.println("[BulletStack]: " + vectors.length + " vectors, " + matrices.length + " matrices, " + quatPosition + " quads, " + transformPosition + " transforms");
    }

    static {
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

    public static Vector3d newVec() {
        Vector3d[] vectors = Stack.vectors;
        if (vectorPosition >= vectors.length) {
            int newSize = vectors.length * 2;
            Vector3d[] values = new Vector3d[newSize];
            System.arraycopy(vectors, 0, values, 0, vectors.length);
            for (int i = vectors.length; i < newSize; i++) {
                values[i] = new Vector3d();
            }
            Stack.vectors = vectors = values;
        }
        return vectors[vectorPosition++];
    }

    public static Quat4d newQuat() {
        Quat4d[] quads = Stack.quads;
        if (quatPosition >= quads.length) {
            int newSize = quads.length * 2;
            Quat4d[] values = new Quat4d[newSize];
            System.arraycopy(quads, 0, values, 0, quads.length);
            for (int i = quads.length; i < newSize; i++) {
                values[i] = new Quat4d();
            }
            Stack.quads = quads = values;
        }
        return quads[quatPosition++];
    }

    public static Matrix3d newMat(Matrix3d base) {
        Matrix3d v = newMat();
        v.set(base);
        return v;
    }

    public static Matrix3d newMat() {
        Matrix3d[] matrices = Stack.matrices;
        if (matrixPosition >= matrices.length) {
            int newSize = matrices.length * 2;
            Matrix3d[] values = new Matrix3d[newSize];
            System.arraycopy(matrices, 0, values, 0, matrices.length);
            for (int i = matrices.length; i < newSize; i++) {
                values[i] = new Matrix3d();
            }
            Stack.matrices = matrices = values;
        }
        Matrix3d m = matrices[matrixPosition++];
        if (m == null) throw new RuntimeException();
        return m;
    }

    public static Transform newTrans(Transform base) {
        Transform v = newTrans();
        v.set(base);
        return v;
    }

    public static Transform newTrans() {
        Transform[] transforms = Stack.transforms;
        if (transformPosition >= transforms.length) {
            int newSize = transforms.length * 2;
            Transform[] values = new Transform[newSize];
            System.arraycopy(transforms, 0, values, 0, transforms.length);
            for (int i = transforms.length; i < newSize; i++) {
                values[i] = new Transform();
            }
            Stack.transforms = transforms = values;
        }
        Transform m = transforms[transformPosition++];
        if (m == null) throw new RuntimeException();
        return m;
    }

    public static ManifoldPoint newManifoldPoint() {
        ManifoldPoint[] manifoldPoints = Stack.manifoldPoints;
        if (manifoldPosition >= manifoldPoints.length) {
            int newSize = manifoldPoints.length * 2;
            ManifoldPoint[] values = new ManifoldPoint[newSize];
            System.arraycopy(manifoldPoints, 0, values, 0, manifoldPoints.length);
            for (int i = manifoldPoints.length; i < newSize; i++) {
                values[i] = new ManifoldPoint();
            }
            Stack.manifoldPoints = manifoldPoints = values;
        }
        return manifoldPoints[manifoldPosition++];
    }

    public static Vector3d borrowVec() {
        Vector3d v = newVec();
        vectorPosition--;
        return v;
    }

    public static Vector3d borrowVec(Vector3d set) {
        Vector3d v = newVec();
        v.set(set);
        vectorPosition--;
        return v;
    }

    public static Quat4d borrowQuat() {
        Quat4d v = newQuat();
        vectorPosition--;
        return v;
    }

    public static Matrix3d borrowMat() {
        Matrix3d v = newMat();
        matrixPosition--;
        return v;
    }

    public static Matrix3d borrowMat(Matrix3d set) {
        Matrix3d v = newMat();
        v.set(set);
        matrixPosition--;
        return v;
    }

    public static Transform borrowTrans() {
        Transform v = newTrans();
        transformPosition--;
        return v;
    }

    public static Vector3d newVec(Vector3d src) {
        Vector3d value = newVec();
        value.set(src);
        return value;
    }

    // looks slow, we should override all samples...
    public static <T> T alloc(Class<T> clazz) {
        try {
            return clazz.getConstructor().newInstance();
        } catch (InstantiationException | IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }

    public static <T> T alloc(T src) {
        try {
            Class<T> clazz = (Class<T>) src.getClass();
            return clazz.getConstructor(clazz).newInstance(src);
        } catch (InstantiationException | IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }

    public static void libraryCleanCurrentThread() {

    }

}
