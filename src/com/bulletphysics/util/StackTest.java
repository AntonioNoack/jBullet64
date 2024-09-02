package com.bulletphysics.util;

import cz.advel.stack.Stack;

public class StackTest {
    public static void main(String[] args) {
        Stack.newVec();
        Stack.newVec(1);
        Stack.newVec(1, 2, 3);
        Stack.subVec(1);
        Stack.borrowVec();
        sub();
    }

    static void sub() {
        Stack.subVec(1);
    }
}
