package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbSet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climb;
  private final DoubleSupplier m_target1;
  private final double m_time;
  private double m_finishedTime = -1;

  /**
   * Gradually adjust speed? TODO: add this in
   */
  // private final SlewRateLimiter m_limiter1 = new SlewRateLimiter(100);

  public ClimbSet(Climber subsystem, DoubleSupplier climbAngle) {
    m_climb = subsystem;
    m_target1 = climbAngle;
    m_time = -1;

    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.setClimber(m_target1.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
}
