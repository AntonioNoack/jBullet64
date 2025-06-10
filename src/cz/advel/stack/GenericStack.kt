package cz.advel.stack;

import java.nio.BufferUnderflowException;
import java.util.ArrayList;
import java.util.function.Supplier;

import static cz.advel.stack.Stack.limit;

/**
 * this class is now fully thread safe :)
 * it is used to quickly borrow instances from specific classes used in bullet (javax.vecmath)
 */
public class GenericStack<T> {

    public static final ArrayList<GenericStack<?>> STACKS = new ArrayList<>();

    private final String name;
    private final Supplier<T> generator;

    public GenericStack(Supplier<T> generator, String name) {
        this.name = name;
        this.generator = generator;
        synchronized (STACKS) {
            STACKS.add(this);
        }
    }

    private class GenericStackInstance {
        private int position = 0;
        @SuppressWarnings("unchecked")
        private T[] instances = (T[]) new Object[32];
        int depth = 0;

        {
            for (int i = 0, l = instances.length; i < l; i++) {
                instances[i] = generator.get();
            }
        }

        // I either didn't find the library, or it was too large for my liking:
        // I rewrote the main functionalities

        public void reset2(boolean printSlack) {
            if (printSlack) {
                System.out.println("[BulletStack]: Slack: " +
                        position + " " + name
                );
            }
            position = 0;
        }

        public void reset2(int newPosition) {
            position = newPosition;
        }

        public void printSizes2() {
            System.out.println("[BulletStack]: " +
                    instances.length + " " + name);
        }

        public T newInstance() {
            T[] vts = instances;
            if (position >= vts.length) {
                int newSize = vts.length * 2;
                checkLeaking(newSize);
                //noinspection unchecked
                T[] values = (T[]) new Object[newSize];
                System.arraycopy(vts, 0, values, 0, vts.length);
                for (int i = vts.length; i < newSize; i++) {
                    values[i] = generator.get();
                }
                instances = vts = values;
            }
            return vts[position++];
        }
    }

    private final ThreadLocal<GenericStackInstance> instances = ThreadLocal.withInitial(GenericStackInstance::new);

    public void reset(boolean printSlack) {
        instances.get().reset2(printSlack);
    }

    public void reset(int newPosition) {
        instances.get().reset2(newPosition);
    }

    public int getPosition() {
        return instances.get().position;
    }

    public int[] getPosition(int[] dst) {
        if (dst == null) return getPosition(new int[1]);
        GenericStackInstance instance = instances.get();
        dst[0] = instance.position;
        // System.out.println("Getting state [" + instance.depth + "] at " + Arrays.toString(dst));
        instance.depth++;
        return dst;
    }

    public void reset(int[] positions) {
        GenericStackInstance instance = instances.get();
        instance.position = positions[0];
        instance.depth--;
    }

    private static void checkUnderflow(int position) {
        if (position < 0) throw new BufferUnderflowException();
    }

    public void release(int delta) {
        GenericStackInstance stack = instances.get();
        stack.position -= delta;
        printCaller("subVec(d)", 2, stack.position);
        checkUnderflow(stack.position);
    }

    public void printSizes() {
        instances.get().printSizes2();
    }

    private void checkLeaking(int newSize) {
        if (newSize > limit) throw new OutOfMemoryError("Reached stack limit " + limit + ", probably leaking");
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

    public T create() {
        GenericStackInstance stack = instances.get();
        printCaller("newVec()", 2, stack.position);
        return stack.newInstance();
    }
}
