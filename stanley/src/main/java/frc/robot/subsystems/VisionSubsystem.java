package frc.robot.subsystems;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import WallEye.*;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetGyroCommand;

public class VisionSubsystem extends MeasurableSubsystem {
    private WallEye wallEye;
    private WallEyeResult[] results;
    private Pose3d camOnePose = new Pose3d();
    private int numCams = 1;
    private int updates = 0;
    private int numTags = 0;
    private double camOneDelay = 0;
    private double tagOneAmbig = 0;
    public static DriveSubsystem driveSubsystem;
    private int[] dios = {};
    private Transform3d camToRobot = new Transform3d(new Translation3d(-0.2, 0.0, 0.0), new Rotation3d());

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
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
                if (res.getAmbiguity() < 0.15 || res.getNumTags() > 1)
                    driveSubsystem.updateOdometryWithVision(wallEye.camPoseToCenter(0, res.getCameraPose()).toPose2d(), res.getTimeStamp()/1000000);
            }
        }
        catch (Exception e)
        {}
        }

    }

    private Pose2d camToRobot(Pose2d camPose, Pose2d camToRobot) {
        return camPose.transformBy(camToRobot.minus(new Pose2d()));
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
    
    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("Cam x", () -> camOnePose.getX()), 
            new Measure("Cam y", () -> camOnePose.getY()), 
            new Measure("Cam z", () -> camOnePose.getZ()),
            new Measure("latency", () -> camOneDelay/1000),
            new Measure("Update num", () -> updates), 
            new Measure("Tag Ambig", () -> tagOneAmbig));
    }
}
