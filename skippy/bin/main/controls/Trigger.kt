package org.strykeforce.controls

import mu.KotlinLogging
import org.slf4j.LoggerFactory

/** Detects triggering events for polled inputs. For example, detecting a button "down" event.  */
abstract class Trigger {
    private val logger = KotlinLogging.logger {}

    private var isActiveLast = false

    /**
     * This will return true upon the first call after the input is active. It resets when the input
     * is deactivated.
     *
     * @return true when input is first activated
     */
    val isActivated: Boolean
        get() {
            if (get()) {
                if (!isActiveLast) {
                    isActiveLast = true
                    logger.debug("{} has activated", this)
                    return true
                }
            } else {
                isActiveLast = false
            }
            return false
        }

    /**
     * Subclasses implement this to signal when a input is active. It usually is polled by a config
     * loop.
     *
     * @return true if the trigger is active
     */
    abstract fun get(): Boolean
}
