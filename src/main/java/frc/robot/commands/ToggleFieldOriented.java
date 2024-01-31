package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleFieldOriented extends InstantCommand {
  private final DriveSubsystem m_drive;

  public ToggleFieldOriented(DriveSubsystem subsystem) {
    m_drive = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_drive.setIsFieldOriented(!m_drive.getIsFieldOriented());
  }

}
