package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Inout;

/**
 * Intakes a note.
 */
public class IntakeTime extends Command {
  private final Inout m_inout;
  private final double m_time, m_speed;
  private double m_startTime;

  public IntakeTime(Inout inout, double time, double speed) {
    m_inout = inout;
    m_time = time;
    m_speed = speed;

    addRequirements(inout);
  }

  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_inout.setIntake(m_speed);
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_time;
  }

  @Override
  public void end(boolean interrupted) {
    m_inout.setIntake(0);
  }
}
