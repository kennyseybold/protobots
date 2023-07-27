// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.GimpDriveCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick driveJoystick = new Joystick(0);
  private DriveSubsystem driveSubsystem;
  private TelemetryService telemetryService;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    // Configure the button bindings
    configureButtonBindings();

    telemetryService = new TelemetryService(TelemetryController::new);
    driveSubsystem.registerWith(telemetryService);

    driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveJoystick, driveSubsystem));
    telemetryService.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveJoystick.setXChannel(1);
    driveJoystick.setYChannel(0);
    driveJoystick.setTwistChannel(2);

    new JoystickButton(driveJoystick, Button.A.id)
        .whenPressed(new InstantCommand(driveSubsystem::resetGyro, driveSubsystem));
    new JoystickButton(driveJoystick, Button.X.id)
        .whenPressed(new InstantCommand(driveSubsystem::xLock, driveSubsystem));
    new JoystickButton(driveJoystick, Button.START.id)
        .whenPressed(new GimpDriveCommand(driveSubsystem));
  }

  public enum Button {
    A(2),
    B(3),
    Y(4),
    X(1),
    START(10);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }
}
