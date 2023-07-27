package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {
  private final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private final SwerveDrive swerveDrive;
  public boolean isGimp = true;

  public DriveSubsystem() {
    var moduleBuilder =
        new TalonSwerveModule.Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

    TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();
    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
      azimuthTalon.configAllSettings(
          DriveConstants.getAzimuthTalonConfig(), Constants.kTalonConfigTimeout);
      azimuthTalon.configSupplyCurrentLimit(
          DriveConstants.getAzimuthCurrentLimit(), Constants.kTalonConfigTimeout);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);
      azimuthTalon.setInverted(true);

      var driveTalon = new TalonSRX(i + 10);
      driveTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
      driveTalon.configAllSettings(
          DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
      driveTalon.configSupplyCurrentLimit(
          DriveConstants.getDriveCurrentLimit(), Constants.kTalonConfigTimeout);
      driveTalon.enableVoltageCompensation(true);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .build();

      loadAndSetAzimuthZeroReference(swerveModules[i], i);
    }
    swerveDrive = new SwerveDrive(swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0.0));
  }

  private void loadAndSetAzimuthZeroReference(TalonSwerveModule module, int i) {
    int index = i;
    String key = String.format("SwerveDrive/wheel.%d", index);
    int reference = Preferences.getInt(key, Integer.MIN_VALUE);
    if (reference == Integer.MIN_VALUE) {
      logger.error("no saved azimuth zero reference for swerve module {}", index);
    }
    int azimuthAbsoluteCounts =
        module.getAzimuthTalon().getSensorCollection().getPulseWidthPosition() & 0xFFF;

    int azimuthSetpoint = reference - azimuthAbsoluteCounts;
    ErrorCode errorCode =
        module.getAzimuthTalon().setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    if (errorCode.value != 0) {
      logger.error("Talon error code while setting azimuth zero: {}", errorCode);
    }

    module.getAzimuthTalon().set(ControlMode.MotionMagic, azimuthSetpoint);
    logger.info(
        "swerve module {}: azimuth zero reference = {}, absolute position = {}, current position = {}",
        index,
        reference,
        azimuthAbsoluteCounts,
        azimuthSetpoint);
  }

  public void drive(double fwdMeterPerSec, double strMeterPerSec, double yawRadPerSec) {
    swerveDrive.drive(fwdMeterPerSec, strMeterPerSec, yawRadPerSec, true);
  }

  public void move(
      double fwdMeterPerSec, double strMeterPerSec, double yawRadPerSec, Boolean isFieldOriented) {
    swerveDrive.move(fwdMeterPerSec, strMeterPerSec, yawRadPerSec, isFieldOriented);
  }

  @Override
  public void periodic() {
    swerveDrive.periodic();
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public void lockZero() {
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(0.0));
    }
    logger.info("Locking wheels to zero");
  }

  public void xLock() {
    SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      if (i == 1 || i == 2) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(-45.0));
      } else {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(45.0));
      }
    }
    logger.info("X-locking wheels");
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Pose2d getPoseMeters() {
    return swerveDrive.getPoseMeters();
  }

  public double getMaxSpeed() {
    if (isGimp) {
      return Constants.DriveConstants.kMaxGimpSpeedMetersPerSecond;
    } else {
      return Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    }
  }

  public double getMaxOmega() {
    if (isGimp) {
      return Constants.DriveConstants.kMaxGimpOmega;
    } else {
      return Constants.DriveConstants.kMaxOmega;
    }
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    swerveDrive.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Gyro Rotation2D(deg)", () -> swerveDrive.getHeading().getDegrees()),
        new Measure("Odometry X", () -> swerveDrive.getPoseMeters().getX()),
        new Measure("Odometry Y", () -> swerveDrive.getPoseMeters().getY()));
  }
}
