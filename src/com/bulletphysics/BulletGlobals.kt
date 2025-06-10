package com.bulletphysics

import com.bulletphysics.util.ArrayPool
import com.bulletphysics.util.ObjectPool
import cz.advel.stack.Stack

/**
 * Bullet global settings and constants.
 *
 * @author jezek2
 */
@Suppress("unused")
object BulletGlobals {
    var DEBUG: Boolean = false

    const val CONVEX_DISTANCE_MARGIN: Double = 0.04

    // we may have to change that to the correct double value
    const val FLT_EPSILON: Double = 2.220446049250313E-16
    const val SIMD_EPSILON: Double = FLT_EPSILON

    const val SIMD_2_PI: Double = Math.PI * 2.0
    const val SIMD_PI: Double = Math.PI
    const val SIMD_HALF_PI: Double = SIMD_2_PI * 0.25
    const val SIMD_RADS_PER_DEG: Double = SIMD_2_PI / 360.0
    const val SIMD_DEGS_PER_RAD: Double = 360.0 / SIMD_2_PI
    const val SIMD_INFINITY: Double = Double.Companion.MAX_VALUE

    /** ///////////////////////////////////////////////////////////////////////// */
    private val INSTANCES = ThreadLocal.withInitial { Globals() }

    var contactAddedCallback: ContactAddedCallback?
        get() = INSTANCES.get().gContactAddedCallback
        set(callback) {
            INSTANCES.get().gContactAddedCallback = callback
        }

    var contactDestroyedCallback: ContactDestroyedCallback?
        get() = INSTANCES.get().gContactDestroyedCallback
        set(callback) {
            INSTANCES.get().gContactDestroyedCallback = callback
        }

    var contactProcessedCallback: ContactProcessedCallback?
        get() = INSTANCES.get().gContactProcessedCallback
        set(callback) {
            INSTANCES.get().gContactProcessedCallback = callback
        }

    var contactBreakingThreshold: Double
        /** ///////////////////////////////////////////////////////////////////////// */
        get() = INSTANCES.get().contactBreakingThreshold
        set(threshold) {
            INSTANCES.get().contactBreakingThreshold = threshold
        }

    var deactivationTime: Double
        get() = INSTANCES.get().deactivationTime
        set(time) {
            INSTANCES.get().deactivationTime = time
        }

    var isDeactivationDisabled: Boolean
        get() = INSTANCES.get().disableDeactivation
        set(disable) {
            INSTANCES.get().disableDeactivation = disable
        }

    /**
     * Cleans all current thread specific settings and caches.
     */
    fun cleanCurrentThread() {
        INSTANCES.remove()
        Stack.libraryCleanCurrentThread()
        ObjectPool.cleanCurrentThread()
        ArrayPool.cleanCurrentThread()
    }

    private class Globals {
        var gContactDestroyedCallback: ContactDestroyedCallback? = null
        var gContactAddedCallback: ContactAddedCallback? = null
        var gContactProcessedCallback: ContactProcessedCallback? = null

        var contactBreakingThreshold = 0.02

        // RigidBody
        var deactivationTime = 2.0
        var disableDeactivation = false
    }
}
