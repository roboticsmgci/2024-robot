package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Instant command that toggles between field oriented and robot oriented driving.
 */
public class ToggleFieldOriented extends InstantCommand {
  private final SwerveSubsystem m_drive;

  public ToggleFieldOriented(SwerveSubsystem subsystem) {
    m_drive = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_drive.setIsFieldOriented(!m_drive.getIsFieldOriented());
  }

}
