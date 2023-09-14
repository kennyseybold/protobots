package frc.robot.subsystems;

import java.util.Set;

import org.intellij.lang.annotations.JdkConstants.CursorType;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

import WallEye.*;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetGyroCommand;

public class VisionSubsystem extends MeasurableSubsystem {
    private WallEye wallEye;
    private WallEyeResult[] results;
    private Pose3d camOnePose = new Pose3d();
    private int numCams = 1;
    private int updates = 0;
    private int numTags = 0;
    private int visionUpdateNum = 0;
    private double camOneDelay = 0;
    private double tagOneAmbig = 0;
    public static DriveSubsystem driveSubsystem;
    private DigitalInput[] dios = {new DigitalInput(0)};
    private Transform3d camToRobot = new Transform3d(new Translation3d(-0.2, 0.0, 0.0), new Rotation3d());
    private Pose2d suppliedCamPose = new Pose2d();
    private boolean trustingWheels = true;
    private VisionStates curState = VisionStates.trustWheels;
    private DigitalInput temp = dios[0];
    private double timeLastCam = 0.0;
    private int timesCamOffWheel = 0;
    private int numUpdateForReset = 0;

    public Matrix<N3, N1> adaptiveVisionMatrix;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        adaptiveVisionMatrix = DriveConstants.kVisionMeasurementStdDevs.copy();
        this.driveSubsystem = driveSubsystem;
        wallEye = new WallEye("WallEye", numCams, () -> driveSubsystem.getYaw(), dios);
        wallEye.setCamToCenter(0, camToRobot);
    }

    @Override
    public void periodic() {
        if (wallEye.hasNewUpdate())
        {
            results = wallEye.getResults();
            camOneDelay = RobotController.getFPGATime() - results[0].getTimeStamp();
            numTags = results[0].getNumTags();
            camOnePose = wallEye.camPoseToCenter(0, results[0].getCameraPose());
            updates = wallEye.getUpdateNumber();
            tagOneAmbig = results[0].getAmbiguity();
            try 
            {
                for(WallEyeResult res: results)
                {
                    Pose2d camPose = wallEye.camPoseToCenter(0, res.getCameraPose()).toPose2d();
                    if ((res.getAmbiguity() < 0.15 || res.getNumTags() > 1))
                    {
                        timeLastCam = getTimeSec();
                        switch (curState) {
                            case trustWheels:
                                if (canAcceptPose(camPose)) {
                                    handleVision(res);
                                    timesCamOffWheel = 0;
                                } else {
                                    timesCamOffWheel++;
                                }
                                if (numUpdateForReset > DriveConstants.kNumResultsToResetStdDev) 
                                    adaptiveVisionMatrix = DriveConstants.kVisionMeasurementStdDevs.copy();

                                break;
                            case trustVision:
                                handleVision(res);
                                if (numUpdateForReset > DriveConstants.kNumResultsToTrustWheels) {
                                    curState = VisionStates.trustWheels;
                                    adaptiveVisionMatrix = DriveConstants.kVisionMeasurementStdDevs.copy();
                                }

                                break;
                            case onlyTrustVision:

                                break;
                            case onlyTrustWheels:
                                
                                break;
                        }
                    }
                }
        }
        catch (Exception e)
        {}
        } else {
            if (getTimeSec() - timeLastCam > DriveConstants.kTimeToTightenStdDev) {
                if (getTimeSec() - timeLastCam > DriveConstants.kTimeToTrustCamera)
                    curState = VisionStates.trustVision;

                numUpdateForReset = 0;
                for(int i = 0; i < 2; ++i) {
                    double estimatedWeight = DriveConstants.kVisionMeasurementStdDevs.get(i, 0) 
                        - DriveConstants.kStdDevDecayCoeff * ((getTimeSec() - timeLastCam) - DriveConstants.kTimeToTightenStdDev);
                    
                    adaptiveVisionMatrix.set(i, 0, estimatedWeight < DriveConstants.kMinimumStdDev ? DriveConstants.kMinimumStdDev : estimatedWeight);
                }
            }
        }

    }

    public void handleVision(WallEyeResult res) {
        numUpdateForReset++;
        suppliedCamPose = res.getCameraPose().toPose2d();
        driveSubsystem.updateOdometryWithVision(suppliedCamPose, res.getTimeStamp()/1000000);
        visionUpdateNum++;
    }

    public double getTimeSec() {
        return (double)RobotController.getFPGATime() / 1000000;
    }

    public void switchVisionMode(boolean doTrustWheels) {
        trustingWheels = doTrustWheels;
        curState = trustingWheels ? VisionStates.trustWheels : curState;
    }

    public boolean trustWheels() {
        return trustingWheels;
    }

    private boolean canAcceptPose(Pose2d cam) {
        ChassisSpeeds speed = driveSubsystem.getFieldRelSpeed();
        Pose2d curPose = driveSubsystem.getPoseMeters();
        Transform2d disp = curPose.minus(cam);
        double magnitudeVel = Math.sqrt(Math.pow(speed.vxMetersPerSecond, 2) + Math.pow(speed.vyMetersPerSecond, 2));
        double magnitudeDisp = Math.sqrt(Math.pow(disp.getX(), 2) + Math.pow(disp.getY(), 2));
        return (magnitudeDisp < ((magnitudeVel * .1) + .2 + Math.pow((magnitudeVel * .2), 2)));
    }

    public WallEyeResult[] getPoses() {
        return results;
    }
    
    public Pose2d[] getPose2ds() {
        Pose2d[] poses = new Pose2d[numCams];
        for(int i  = 0; i < numCams; ++i)
        {
            poses[i] = results[i].getCameraPose().toPose2d();
        }
        return poses;
    }
    
    public double getChannel0() {
        return temp.get() ? 1.0 : 0.0;
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("Cam x", () -> camOnePose.getX()), 
            new Measure("Cam y", () -> camOnePose.getY()), 
            new Measure("Cam z", () -> camOnePose.getZ()),
            new Measure("latency", () -> camOneDelay/1000),
            new Measure("Update num", () -> updates), 
            new Measure("Tag Ambig", () -> tagOneAmbig), 
            new Measure("Vision Update Num", () -> visionUpdateNum),
            new Measure("Supplied Camera Pose X", () -> suppliedCamPose.getX()),
            new Measure("Supplied Camera Pose Y", () -> suppliedCamPose.getY()),
            new Measure("Dio port 0", ()-> getChannel0()),
            new Measure("Vision State", () -> curState.ordinal()),
            new Measure("X Y standard devs for vision", () -> adaptiveVisionMatrix.get(0, 0)));
    }

    public enum VisionStates {
        trustWheels,
        trustVision,
        onlyTrustVision,
        onlyTrustWheels,
    }
}
