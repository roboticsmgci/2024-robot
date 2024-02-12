package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Inout;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class InoutDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Inout m_inout;
  private final DoubleSupplier m_intake, m_shooter;

  public InoutDrive(Inout subsystem, DoubleSupplier intake, DoubleSupplier shooter) {
    m_inout = subsystem;
    m_intake = intake;
    m_shooter = shooter;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
