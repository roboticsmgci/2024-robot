package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class LockToTarget extends Command {
  
  private final DriveSubsystem m_drive;
  private final Translation2d m_targetLocation;
  private final DoubleSupplier m_xControl, m_yControl;
  private final SlewRateLimiter m_xSlewRateLimiter = new SlewRateLimiter(DriverConstants.kHorizontalRateLimit);
  private final SlewRateLimiter m_ySlewRateLimiter = new SlewRateLimiter(DriverConstants.kHorizontalRateLimit);
  private final PIDController m_rotSpeedPID = new PIDController(
    PIDConstants.kPLockTargetRotSpeed,
    PIDConstants.kILockTargetRotSpeed,
    PIDConstants.kDLockTargetRotSpeed
  );

  public LockToTarget(DriveSubsystem subsystem, Translation2d targetLocation, DoubleSupplier xControl, DoubleSupplier yControl) {
    m_drive = subsystem;
    m_targetLocation = targetLocation;
    m_xControl = xControl;
    m_yControl = yControl;

    m_rotSpeedPID.enableContinuousInput(-180, 180);

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_drive.drive(
        controlToSpeed(m_xSlewRateLimiter.calculate(m_xControl.getAsDouble()), DriverConstants.kMaxHorizontalSpeed),
        controlToSpeed(m_ySlewRateLimiter.calculate(m_yControl.getAsDouble()), DriverConstants.kMaxHorizontalSpeed),
        m_rotSpeedPID.calculate(m_drive.getRobotHeadingDegrees(), getDesiredHeading(m_drive.getPose(), m_targetLocation)));
  }

  /**
   * Converts a controller value to a speed.
   * 
   * @param controlValue the controller value; should be in [-1, 1]
   * @param maxSpeed     the maximum speed
   * @return             the speed corresponding to the controller value
   */
  private double controlToSpeed(double controlValue, double maxSpeed) {
    if (Math.abs(controlValue) <= DriverConstants.kControllerDeadzone) {
      return 0;
    } else {
      // Scale the value to occupy the full space between 0-1
      return Math.signum(controlValue) * Math.abs(controlValue - DriverConstants.kControllerDeadzone)
          / (1 - DriverConstants.kControllerDeadzone) * maxSpeed;
    }
  }

  private double getDesiredHeading(Pose2d currentPose, Translation2d targetLocation) {
    return Math.toDegrees(Math.atan2(targetLocation.getY() - currentPose.getY(), targetLocation.getX() - currentPose.getX()));
  }

}
