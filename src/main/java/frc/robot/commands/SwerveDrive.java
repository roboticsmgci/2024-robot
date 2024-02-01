package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends Command {
  
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_xControl, m_yControl, m_rotControl;
  private final SlewRateLimiter m_xSlewRateLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_ySlewRateLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_rotSlewRateLimiter = new SlewRateLimiter(3);

  /**
   * Constructs a <code>SwerveDrive</code> command.
   * 
   * @param subsystem the drive subsystem
   * @param xControl the controller input for the x speed; the output should be in [-1, 1]
   * @param yControl the controller input for the y speed; the output should be in [-1, 1]
   * @param rotControl the controller input for the rotational speed; the output should be in [-1, 1]
   */
  public SwerveDrive(DriveSubsystem subsystem, DoubleSupplier xControl, DoubleSupplier yControl, DoubleSupplier rotControl) {
    m_drive = subsystem;
    m_xControl = xControl;
    m_yControl = yControl;
    m_rotControl = rotControl;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_drive.drive(
      controlToSpeed(m_xSlewRateLimiter.calculate(m_xControl.getAsDouble()), DriverConstants.kMaxHorizontalSpeed),
      controlToSpeed(m_ySlewRateLimiter.calculate(m_yControl.getAsDouble()), DriverConstants.kMaxHorizontalSpeed),
      controlToSpeed(m_rotSlewRateLimiter.calculate(m_rotControl.getAsDouble()), DriverConstants.kMaxRotationalSpeed)
    );
  }

  private double controlToSpeed(double controlValue, double maxSpeed) {
    if (Math.abs(controlValue) <= DriverConstants.kControllerDeadzone) {
      return 0;
    } else {
      // Scale the value to occupy the full space between 0-1
      return Math.signum(controlValue) * Math.abs(controlValue - DriverConstants.kControllerDeadzone) / (1 - DriverConstants.kControllerDeadzone) * maxSpeed;
    }
  }

}
