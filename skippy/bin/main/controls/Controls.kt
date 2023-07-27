package org.strykeforce.controls

import edu.wpi.first.wpilibj.Joystick

/** Accesses driver config input.  */
class Controls {

    private val driverController = Joystick(1)

    val resetButton: Trigger
        get() = object : Trigger() {
            override fun get(): Boolean {
                return driverController.getRawButton(DRIVER_RESET_BUTTON)
            }
        }

    /**
     * Return the driver controller left stick Y-axis position.
     *
     * @return the position, range is -1.0 (full reverse) to 1.0 (full forward)
     */
    val forward: Double
        get() = -driverController.getRawAxis(DRIVER_LEFT_Y_AXIS)

    /**
     * Return the driver controller left stick X-axis position.
     *
     * @return the position, range is -1.0 (full left) to 1.0 (full right)
     */
    val strafe: Double
        get() = driverController.getRawAxis(DRIVER_LEFT_X_AXIS)

    /**
     * Return the driver controller right stick X-axis position.
     *
     * @return the position, range is -1.0 (full left) to 1.0 (full right)
     */
    val azimuth: Double
        get() = driverController.getRawAxis(DRIVER_RIGHT_X_AXIS)

}

const val DRIVER_RIGHT_X_AXIS = 0
const val DRIVER_RIGHT_Y_AXIS = 1
const val DRIVER_LEFT_Y_AXIS = 2
const val DRIVER_TUNER_AXIS = 6
const val DRIVER_LEFT_X_AXIS = 5

const val DRIVER_LEFT_BUTTON = 1
const val DRIVER_RIGHT_SHOULDER_BUTTON = 2
const val DRIVER_RESET_BUTTON = 3
const val DRIVER_LEFT_SHOULDER_DOWN_BUTTON = 4
const val DRIVER_LEFT_SHOULDER_UP_BUTTON = 5

