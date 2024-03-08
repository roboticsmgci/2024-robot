package frc.robot.commands;

import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmSet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_arm;
  private final DoubleSupplier m_target1, m_target2;
  private final double m_time;
  private double m_finishedTime = -1;

  private final PIDController m_armSpeedPID1 = new PIDController(0.001, 0, 0);
  private final PIDController m_armSpeedPID2 = new PIDController(0.001, 0, 0);

  /**
   * Gradually adjust speed? TODO: add this in
   */
  // private final SlewRateLimiter m_limiter1 = new SlewRateLimiter(100);

  public ArmSet(Arm subsystem, DoubleSupplier arm1Angle, DoubleSupplier arm2Angle) {
    m_arm = subsystem;
    m_target1 = arm1Angle;
    m_target2 = arm2Angle;
    m_time = -1;

    addRequirements(subsystem);
  }

  public ArmSet(Arm subsystem, DoubleSupplier arm1Angle, DoubleSupplier arm2Angle, double time) {
    m_arm = subsystem;
    m_target1 = arm1Angle;
    m_target2 = arm2Angle;
    m_time = time;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_arm.setArm1(m_limiter1.calculate(m_armSpeedPID1.calculate(m_arm.getArmEncoder1(), m_target1.getAsDouble())));
    m_arm.setArm1(m_armSpeedPID1.calculate(m_arm.getArmEncoder1(), m_target1.getAsDouble()));

    m_arm.setArm2(m_armSpeedPID2.calculate(m_arm.getArmEncoder2(), m_target2.getAsDouble()));

    if (m_finishedTime == -1 && m_armSpeedPID1.atSetpoint() && m_armSpeedPID2.atSetpoint()) {
      m_finishedTime = System.currentTimeMillis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_finishedTime != -1 && m_time != -1 && (System.currentTimeMillis() - m_finishedTime) >= m_time) {
      return true;
    }
    return false;
  }
}
