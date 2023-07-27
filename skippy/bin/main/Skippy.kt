package org.strykeforce

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.TimedRobot
import mu.KotlinLogging
import org.strykeforce.controls.Controls
import org.strykeforce.thirdcoast.swerve.SwerveDrive
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig
import org.strykeforce.thirdcoast.swerve.Wheel
import java.io.File

const val SAVE_AZIMUTHS = "/home/lvuser/zero.me"

const val DEADBAND = 0.05
private val logger = KotlinLogging.logger {}

class Skippy : TimedRobot() {

    private val controls = Controls()
    private val ahrs = AHRS(SPI.Port.kMXP)
    private val swerveDrive = SwerveDrive(
        SwerveDriveConfig().apply {
            wheels = getSkippyWheels()
            gyro = ahrs
            length = 20.625
            width = 26.125
            gyroLoggingEnabled = true
            summarizeTalonErrors = false
        }
    )
    private val gyroResetButton = controls.resetButton

    override fun robotInit() {

        // save azimuth zero positions by creating file named in SAVE_AZIMUTHS
        if (File(SAVE_AZIMUTHS).delete()) swerveDrive.saveAzimuthPositions()
        swerveDrive.zeroAzimuthEncoders()
    }

    override fun teleopInit() {
        swerveDrive.stop()
        logger.debug { "gyro connected: ${ahrs.isConnected}" }
    }

    override fun teleopPeriodic() {
        val forward = controls.forward.applyDeadband(DEADBAND)
        val strafe = controls.strafe.applyDeadband(DEADBAND)
        val azimuth = controls.azimuth.applyDeadband(DEADBAND)

        swerveDrive.drive(forward, strafe, azimuth)

        if (gyroResetButton.isActivated) {
            swerveDrive.gyro.zeroYaw()
            "reset gyro zero".let {
                logger.warn(it)
                DriverStation.reportWarning(it, false)
            }
        }
    }
}

private fun Double.applyDeadband(amount: Double) = if (Math.abs(this) < amount) 0.0 else this

private fun getSkippyWheels(): Array<Wheel> {
    val azimuthConfig = TalonSRXConfiguration().apply {
        primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative
        continuousCurrentLimit = 10
        peakCurrentLimit = 0
        peakCurrentDuration = 0
        slot_0.apply {
            kP = 10.0
            kI = 0.0
            kD = 100.0
            kF = 1.0
            integralZone = 0
            allowableClosedloopError = 0
        }
        motionAcceleration = 10_000
        motionCruiseVelocity = 800
    }


    val driveConfig = TalonSRXConfiguration().apply {
        primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative
        continuousCurrentLimit = 40
        peakCurrentLimit = 0
        peakCurrentDuration = 0
    }

    val timeout = 10

    return Array(4) {
        Wheel(
            TalonSRX(it).apply { configAllSettings(azimuthConfig, timeout) },
            TalonSRX(it + 10).apply {
                configAllSettings(driveConfig, timeout)
                setNeutralMode(NeutralMode.Brake)
            }, 0.0
        )
    }
}



