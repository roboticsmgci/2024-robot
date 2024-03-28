package frc.robot.commands;

import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmSet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_arm;
  private final DoubleSupplier m_target1, m_target2;
  private final double m_time;
  private double m_finishedTime = -1;

  private final PIDController m_armSpeedPID1 = new PIDController(1.0/2, 0.01, 0);
  private final PIDController m_armSpeedPID2 = new PIDController(1.0, 0.05, 0);

  /**
   * Gradually adjust speed? TODO: add this in
   */
  // private final SlewRateLimiter m_limiter1 = new SlewRateLimiter(100);

  public ArmSet(Arm subsystem, DoubleSupplier arm1Angle, DoubleSupplier arm2Angle) {
    m_arm = subsystem;
    m_target1 = arm1Angle;
    m_target2 = arm2Angle;
    m_time = -1;
    m_armSpeedPID1.setIntegratorRange(-1, 0);

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
    // System.out.println("command running");
    double arm1Power, arm2Power;

    if (m_arm.getArmEncoder1() < Math.toRadians(-20) && (m_arm.getAbsoluteArmAngle()) > Math.toRadians(70)) {
      arm1Power = m_armSpeedPID1.calculate(m_arm.getArmEncoder1(), 0);
      arm2Power = m_armSpeedPID2.calculate(m_arm.getArmEncoder2(), Math.toRadians(90));
    } else {
      
      arm1Power = m_armSpeedPID1.calculate(m_arm.getArmEncoder1(), m_target1.getAsDouble());
      arm2Power = m_armSpeedPID2.calculate(m_arm.getArmEncoder2(), m_target2.getAsDouble());
    }

    System.out.println(m_arm.getArmEncoder2() + " " + m_target2.getAsDouble() + " " + arm2Power);

    // System.out.println(m_arm.getArmEncoder1() + " " + m_target1.getAsDouble());
    // System.out.println(test);
    // m_arm.setArm1(m_limiter1.calculate(m_armSpeedPID1.calculate(m_arm.getArmEncoder1(), m_target1.getAsDouble())));
    // System.out.println("test: " + test);
    //use 0.5 when arm stops skipping
    m_arm.setArm1(-MathUtil.clamp(arm1Power, -0.5, 0.5));

    m_arm.setArm2(-MathUtil.clamp(arm2Power, -0.5, 0.5));
    // m_arm.setArm0(0.7);

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

  public boolean atSetpoint(){
    //use commented one
    // return true;
    return (m_armSpeedPID1.atSetpoint() && m_armSpeedPID2.atSetpoint());
  }
}
