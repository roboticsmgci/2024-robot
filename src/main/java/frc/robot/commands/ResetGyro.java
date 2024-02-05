package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Instant command that resets the gyro sensor.
 */
public class ResetGyro extends InstantCommand {
  private final DriveSubsystem m_drive;

  public ResetGyro(DriveSubsystem subsystem) {
    m_drive = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_drive.resetGyro();
  }
}
