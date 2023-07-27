package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
  private final Joystick driveJoystick;
  private final DriveSubsystem driveSubsystem;

  public TeleopDriveCommand(Joystick driveJoystick, DriveSubsystem driveSubsystem) {
    this.driveJoystick = driveJoystick;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    driveSubsystem.drive(
        MathUtil.applyDeadband(driveJoystick.getX(), DriveConstants.kDeadbandAllStick)
            * driveSubsystem.getMaxSpeed(),
        MathUtil.applyDeadband(driveJoystick.getY(), DriveConstants.kDeadbandAllStick)
            * driveSubsystem.getMaxSpeed(),
        MathUtil.applyDeadband(driveJoystick.getTwist(), DriveConstants.kDeadbandAllStick)
            * driveSubsystem.getMaxOmega());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}
