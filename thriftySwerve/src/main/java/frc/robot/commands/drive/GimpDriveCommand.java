package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class GimpDriveCommand extends InstantCommand {
  private final DriveSubsystem driveSubsystem;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public GimpDriveCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    // if (driveSubsystem.isGimp) {
    //     driveSubsystem.isGimp = false;
    // } else {
    //     driveSubsystem.isGimp = true;
    // }
    driveSubsystem.isGimp = !driveSubsystem.isGimp;
    logger.info("Toggling gimp mode: {}", driveSubsystem.isGimp);
  }
}
