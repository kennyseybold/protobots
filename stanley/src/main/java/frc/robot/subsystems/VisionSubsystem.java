package frc.robot.subsystems;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import WallEye.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;

public class VisionSubsystem extends MeasurableSubsystem {
    private WallEye wallEye;
    private WallEyeResult[] results;
    private Pose3d camOnePose = new Pose3d();
    private int numCams = 1;
    private int updates = 0;
    private double camOneDelay = 0;
    private int numTags = 0;
    public VisionSubsystem() {
        wallEye = new WallEye("Walleye", numCams);
    }

    @Override
    public void periodic() {
        results = wallEye.getResults();
        if (camOnePose.getTranslation().getDistance(results[0].getCameraPose().getTranslation()) > 0.01)
        {
            updates++;
            camOneDelay = (double) RobotController.getFPGATime() - results[0].getTimeStamp();
            camOnePose = results[0].getCameraPose();
            
        }
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
            new Measure("Num Tags", () -> numTags));
    }
}
