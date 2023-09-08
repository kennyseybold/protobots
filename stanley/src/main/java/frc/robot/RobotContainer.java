// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.SwitchVisionMode;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private TelemetryService telemetryService = new TelemetryService(TelemetryController::new);
  private Joystick flysky = new Joystick(0);

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem(telemetryService);
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    driveSubsystem.registerWith(telemetryService);
    visionSubsystem.registerWith(telemetryService);
    telemetryService.start();
    
    configureMatchBoard();
    configureBindings();
  }

  private void configureMatchBoard() {
    ShuffleboardTab match = Shuffleboard.getTab("Match");
    match.addBoolean("Trust Wheels", () -> visionSubsystem.trustWheels())
    .withWidget("Boolean").withPosition(1, 1).withSize(1, 1);
    
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new TeleopDriveCommand(flysky, driveSubsystem, 0, 1, 2));
    new JoystickButton(flysky, 12).onTrue(new ResetGyroCommand(driveSubsystem));
    new JoystickButton(flysky, Button.SWD.id).onTrue(new SwitchVisionMode(visionSubsystem, true));
    new JoystickButton(flysky, Button.SWD.id).onFalse(new SwitchVisionMode(visionSubsystem, false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public enum Button {
    SWA(1),
    SWB_UP(2),
    SWB_DWN(3),
    M_SWC(4),
    SWD(5),
    M_SWE(6),
    SWF_UP(7),
    SWF_DWN(8),
    SWG_UP(9),
    SWG_DWN(10),
    M_SWH(11),
    M_LTRIM_UP(12),
    M_LTRIM_DWN(13),
    M_LTRIM_L(14),
    M_LTRIM_R(15),
    M_RTRIM_UP(16),
    M_RTRIM_DWN(17),
    M_RTRIM_L(18),
    M_RTRIM_R(19);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }
}
