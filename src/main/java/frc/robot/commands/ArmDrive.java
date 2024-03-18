package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_arm;
  private final DoubleSupplier m_speed1, m_speed2;
  private final SlewRateLimiter m_limiter = new SlewRateLimiter(ArmConstants.kArm1RateLimit);
  private double prevSpeed = 0;

  public ArmDrive(Arm subsystem, DoubleSupplier speed1, DoubleSupplier speed2) {
    m_arm = subsystem;
    m_speed1 = speed1;
    m_speed2 = speed2;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentSpeed = m_speed1.getAsDouble();

    // if (currentSpeed > prevSpeed) {
    //   currentSpeed = m_limiter.calculate(currentSpeed);
    // } else {
    //   m_limiter.calculate(currentSpeed);
    // }

    m_arm.setArm1(currentSpeed);
    m_arm.setArm2(m_speed2.getAsDouble());
    // m_arm.setArm0(0.7);

    // System.out.println(currentSpeed);

    prevSpeed = currentSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setArm1(0);
    m_arm.setArm2(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
