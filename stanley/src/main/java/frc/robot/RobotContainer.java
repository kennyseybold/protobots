// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private TelemetryService telemetryService = new TelemetryService(TelemetryController::new);
  private Joystick flysky = new Joystick(0);

  public RobotContainer() {
    visionSubsystem = new VisionSubsystem();
    driveSubsystem = new DriveSubsystem(visionSubsystem);
    driveSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    telemetryService.start();

    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new TeleopDriveCommand(flysky, driveSubsystem, 0, 1, 2));
    new JoystickButton(flysky, 12).onTrue(new ResetGyroCommand(driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
