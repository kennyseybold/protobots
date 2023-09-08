package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class SwitchVisionMode extends InstantCommand {

    private static VisionSubsystem visionSubsystem;
    private boolean trustWheels;
    public SwitchVisionMode(VisionSubsystem visionSubsystem, boolean trustWheels) {
        this.visionSubsystem = visionSubsystem;
        this.trustWheels = trustWheels;
    }   

    @Override
    public void initialize() {
        visionSubsystem.switchVisionMode(trustWheels);
    }
}
