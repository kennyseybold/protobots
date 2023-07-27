package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private Joystick joystick;
    private int fwdAxis,strAxis,yawAxis;
    private double[] rawValues = new double[3];

    private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
    private final SlewRateLimiter strLimiter = new SlewRateLimiter(DriveConstants.kRateLimitFwdStr);
    private final SlewRateLimiter yawLimiter = new SlewRateLimiter(DriveConstants.kRateLimitYaw);

    public TeleopDriveCommand(Joystick flysky, DriveSubsystem driveSubsystem, int fwd, int str, int yaw) {
        this.driveSubsystem = driveSubsystem;
        this.joystick = flysky;
        fwdAxis = fwd;
        strAxis = str;
        yawAxis = yaw;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        rawValues[0] = joystick.getRawAxis(fwdAxis);
        rawValues[1] = joystick.getRawAxis(strAxis);
        rawValues[2] = joystick.getRawAxis(yawAxis);
        

        driveSubsystem.drive(
            DriveConstants.kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(rawValues[0], DriveConstants.kDeadbandAllStick),
            -DriveConstants.kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(rawValues[1], DriveConstants.kDeadbandAllStick),
            -DriveConstants.kMaxOmega * MathUtil.applyDeadband(rawValues[2], DriveConstants.kDeadbandAllStick)
        );
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
