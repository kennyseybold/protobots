package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends InstantCommand {
    private DriveSubsystem driveSubsystem;

    public ResetGyroCommand(DriveSubsystem driveSubsystem){
        this.driveSubsystem =driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.resetGyro();
    }
    
}
