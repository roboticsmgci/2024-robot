package frc.robot.commands;

import frc.robot.Constants.InoutConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Inout;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSpeed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Inout m_inout;
  private final double m_speed;

  public IntakeSpeed(Inout subsystem, double speed) {
    m_inout = subsystem;
    m_speed = speed;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_inout.setIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  // When the note is in the correct location (pid?)
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
