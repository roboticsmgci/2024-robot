package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends Command {
  
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_xSpeed, m_ySpeed, m_rotSpeed;

  public SwerveDrive(DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
    m_drive = subsystem;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rotSpeed = rotSpeed;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_drive.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), m_rotSpeed.getAsDouble());
  }

}
