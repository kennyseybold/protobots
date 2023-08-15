package frc.robot.subsystems;

import java.util.Set;

import org.strykeforce.swerve.OdometryStrategy;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class DriveSubsystem extends MeasurableSubsystem {
    private final SwerveDrive swerveDrive;
    private AHRS ahrs;
    private PoseEstimatorOdometryStrategy odometryStrategy;
    private VisionSubsystem visionSubsystem;

    public DriveSubsystem() {
        var moduleBuilder = new TalonSwerveModule.Builder()
        .driveGearRatio(DriveConstants.kDriveGearRatio)
        .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
        .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

        TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
        Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

        for(int i = 0; i < 4; i++){
            var azimuthTalon = new TalonSRX(i);
            azimuthTalon.configFactoryDefault(DriveConstants.kTalonConfigTimeout);
            azimuthTalon.configAllSettings(DriveConstants.getAzimuthTalonConfig(), DriveConstants.kTalonConfigTimeout);
            azimuthTalon.enableCurrentLimit(true);
            azimuthTalon.enableVoltageCompensation(true);
            azimuthTalon.setNeutralMode(NeutralMode.Coast);

            var driveTalon = new TalonFX(i + 10);
            driveTalon.configFactoryDefault(DriveConstants.kTalonConfigTimeout);
            driveTalon.configAllSettings(
                DriveConstants.getDriveTalonConfig(), DriveConstants.kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);

            swerveModules[i] =
                moduleBuilder
                    .azimuthTalon(azimuthTalon)
                    .driveTalon(driveTalon)
                    .wheelLocationMeters(wheelLocations[i])
                    .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond)
                    .build();

            swerveModules[i].loadAndSetAzimuthZeroReference();
        }
        ahrs = new AHRS();
        swerveDrive = new SwerveDrive(ahrs, swerveModules);
        swerveDrive.resetGyro();
        swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));
        odometryStrategy= new PoseEstimatorOdometryStrategy(swerveDrive.getHeading(), new Pose2d(), swerveDrive.getKinematics(), 
            DriveConstants.kStateStdDevs, DriveConstants.kLocalMeasurementStdDevs, DriveConstants.kVisionMeasurementStdDevs, getSwerveModulePositions());
        swerveDrive.setOdometry(odometryStrategy);
    }

    public void updateOdometryWithVision(Pose2d calculatedPose, long timestamp) {
        odometryStrategy.addVisionMeasurement(calculatedPose, timestamp);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
        SwerveModulePosition[] temp = {null, null, null, null};
        for (int i = 0; i < 4; ++i) {
          temp[i] = swerveModules[i].getPosition();
        }
        return temp;
      }

    public void drive(double vXmps, double vYmps, double vOmegaRadps){
        swerveDrive.drive(vXmps, vYmps, vOmegaRadps, true);
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void resetGyro(){
        swerveDrive.resetGyro();
        resetOdometry(new Pose2d(swerveDrive.getPoseMeters().getTranslation(), new Rotation2d()));
    }

    public void xlock(){
        SwerveModule[] swerveModules = swerveDrive.getSwerveModules();
        for (int i = 0; i < 4; i++) {
            if (i == 0 || i == 3) {
                swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(45.0));
            }
            if (i == 1 || i == 2) {
                swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(-45.0));
            }
        }
    }

    @Override
    public void periodic() {
        swerveDrive.periodic();
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(this);
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("Odom X", () -> swerveDrive.getPoseMeters().getX()), new Measure("Odom Y", () -> swerveDrive.getPoseMeters().getY()));
    }
    
}
