package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InoutConstants;
import frc.robot.subsystems.Inout;

/**
 * Shoots a note.
 */
public class Shoot extends Command {
  private final Inout m_inout;
  private double m_startTime, m_speed;

  public Shoot(Inout inout, double speed) {
    m_inout = inout;
    m_speed = speed;

    addRequirements(inout);
  }

  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    // System.out.println(InoutConstants.kInitialShooterSpeed + ((InoutConstants.kShooterSpeed - InoutConstants.kInitialShooterSpeed) * ((System.currentTimeMillis() - m_startTime)/InoutConstants.kWarmupTime)));
    // m_inout.setShooter(InoutConstants.kInitialShooterSpeed + ((InoutConstants.kShooterSpeed - InoutConstants.kInitialShooterSpeed) * ((System.currentTimeMillis() - m_startTime)/InoutConstants.kWarmupTime)));
    m_inout.setShooter(m_speed);
    
    if ((System.currentTimeMillis() - m_startTime) > InoutConstants.kWarmupTime) {
      m_inout.setIntake(m_speed);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_inout.setIntake(0);
    m_inout.setShooter(0);
  }
}
