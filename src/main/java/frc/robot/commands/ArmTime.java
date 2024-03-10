package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/**
 * Intakes a note.
 */
public class ArmTime extends Command {
  private final Arm m_arm;
  private final double m_time, m_speed;
  private double m_startTime;

  public ArmTime(Arm arm, double time, double speed) {
    m_arm = arm;
    m_time = time;
    m_speed = speed;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_arm.setArm1(m_speed);
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_time;
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setArm1(0);
  }
}
