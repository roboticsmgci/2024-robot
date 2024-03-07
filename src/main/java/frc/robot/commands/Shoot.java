package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InoutConstants;
import frc.robot.subsystems.Inout;

/**
 * Shoots a note.
 */
public class Shoot extends Command {
  private final Inout m_inout;
  private double m_startTime;

  public Shoot(Inout inout, double time, double speed) {
    m_inout = inout;

    addRequirements(inout);
  }

  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    m_inout.setShooter(InoutConstants.kShooterSpeed);
    if ((System.currentTimeMillis() - m_startTime) > InoutConstants.kWarmupTime) {
      m_inout.setIntake(InoutConstants.kIntakeSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_inout.setIntake(0);
  }
}
