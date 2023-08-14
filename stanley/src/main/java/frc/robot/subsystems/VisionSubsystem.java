package frc.robot.subsystems;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import WallEye.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;

public class VisionSubsystem extends MeasurableSubsystem {
    private WallEye wallEye;
    private WallEyeResult[] results;
    private Pose3d camOnePose = new Pose3d();
    private int numCams = 1;
    private int updates = 0;
    private double camOneDelay = 0;
    public static DriveSubsystem driveSubsystem;
    private Pose2d camToRobot = new Pose2d(new Translation2d(-0.5, 0), new Rotation2d());

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        wallEye = new WallEye("Walleye", numCams);
    }

    @Override
    public void periodic() {
        if (wallEye.hasNewUpdate())
        {
            results = wallEye.getResults();
            for(WallEyeResult res: results)
            {
                driveSubsystem.updateOdometryWithVision(camToRobot(res.getCameraPose().toPose2d(), camToRobot), (long)res.getTimeStamp());
            }
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
            new Measure("Update num", () -> updates));
    }
}
