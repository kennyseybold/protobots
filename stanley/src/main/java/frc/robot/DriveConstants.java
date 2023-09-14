package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final int kTalonConfigTimeout = 10;
    public static final double kMaxSpeedMetersPerSecond = 5.44;
    public static final double kWheelDiameterInches = 3.0 * (367 / 500.0);
    
    static final double kDriveMotorOutputGear = 30; // practice bot: 22
    static final double kDriveInputGear = 44; // 48
    static final double kBevelInputGear = 15;
    static final double kBevelOutputGear = 45; // 45


    // Drive Base Size and Gearing
    public static final double kRobotWidth = 0.495; // practice bot: 0.625 //Old: .5
    public static final double kRobotLength = 0.62; // practice bot: 0.625 //Old:.615

    public static Translation2d[] getWheelLocationMeters() {
        final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
        final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
        Translation2d[] locs = new Translation2d[4];
        locs[0] = new Translation2d(x, y); // left front
        locs[1] = new Translation2d(x, -y); // right front
        locs[2] = new Translation2d(-x, y); // left rear
        locs[3] = new Translation2d(-x, -y); // right rear
        return locs;
      }

    // Drive Base Size and Gearing

    public static final double kMaxOmega =
        (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
            / 2.0; // wheel locations below

    public static final double kDriveGearRatio =
        (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);


    // Teleop Drive Constants
    //    public static final double kDeadbandXLock = 0.2;
    public static final double kDeadbandAllStick = 0.075;
    //    public static final double kCloseEnoughTicks = 10.0;
    public static final double kRateLimitFwdStr = 3.5; // 2
    public static final double kRateLimitYaw = 3; // 3
    //    public static final double kExpoScaleMoveFactor = 0.6; // .6
    // public static final double kRateLimitMove = 0.3;
    public static final double kExpoScaleYawFactor = 0.75;

    public static TalonSRXConfiguration getAzimuthTalonConfig() {
        // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
        TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
  
        azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
        azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;
  
        azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
        azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
  
        azimuthConfig.continuousCurrentLimit = 10;
        azimuthConfig.peakCurrentDuration = 0;
        azimuthConfig.peakCurrentLimit = 0;
        azimuthConfig.slot0.kP = 10.0;
        azimuthConfig.slot0.kI = 0.0;
        azimuthConfig.slot0.kD = 100.0;
        azimuthConfig.slot0.kF = 1.0;
        azimuthConfig.slot0.integralZone = 0;
        azimuthConfig.slot0.allowableClosedloopError = 0;
        azimuthConfig.slot0.maxIntegralAccumulator = 0;
        azimuthConfig.motionCruiseVelocity = 800;
        azimuthConfig.motionAcceleration = 10_000;
        azimuthConfig.velocityMeasurementWindow = 64;
        azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
        azimuthConfig.voltageCompSaturation = 12;
        azimuthConfig.voltageMeasurementFilter = 32;
        azimuthConfig.neutralDeadband = 0.04;
        return azimuthConfig;
      }
      // Drive Falcon Config
      public static TalonFXConfiguration getDriveTalonConfig() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.supplyCurrLimit.currentLimit = 40;
        driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
        driveConfig.supplyCurrLimit.triggerThresholdTime = 1.0;
        driveConfig.supplyCurrLimit.enable = true;
        driveConfig.statorCurrLimit.enable = false;
        driveConfig.slot0.kP = 0.16; // 0.16
        driveConfig.slot0.kI = 0.0002;
        driveConfig.slot0.kD = 0.000;
        driveConfig.slot0.kF = 0.047;
        driveConfig.slot0.integralZone = 500;
        driveConfig.slot0.maxIntegralAccumulator = 150_000;
        driveConfig.slot0.allowableClosedloopError = 0;
        driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
        driveConfig.velocityMeasurementWindow = 64;
        driveConfig.voltageCompSaturation = 12;
        driveConfig.neutralDeadband = 0.01;
        driveConfig.voltageMeasurementFilter = 32;
        return driveConfig;
      }

      public static Matrix<N3, N1> kStateStdDevs =
          VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));


      public static Matrix<N1, N1> kLocalMeasurementStdDevs =
          VecBuilder.fill(Units.degreesToRadians(0));

      public static Matrix<N3, N1> kVisionMeasurementStdDevs = 
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    
       public static final Pose2d kOdometryZeroPosBlue =
          new Pose2d(new Translation2d(1.80, 5.097), new Rotation2d());

    //When trusting wheels the amount of times before the decay goes away
    public static int kNumResultsToResetStdDev = 3;

    //Times it takes to trust wheels after trusting vision
    public static int kNumResultsToTrustWheels = 5;

    //Time (seconds) before std dev starts to decay
    public static double kTimeToTightenStdDev = 1.0;

    //Time (seconds) before getting kicked into only trusting camera
    public static double kTimeToTrustCamera = 10.0;

    //The linear rate of change on the std dev
    public static double kStdDevDecayCoeff = 0.01/3.0;

    //Minimum std dev for the declining std dev
    public static double kMinimumStdDev = 0.01;

}
