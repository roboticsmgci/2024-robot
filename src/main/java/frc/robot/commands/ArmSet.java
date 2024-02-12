package frc.robot.commands;

import frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_arm;
  private final DoubleSupplier m_target1, m_target2;

  public ArmSet(Arm subsystem, DoubleSupplier arm1Angle, DoubleSupplier arm2Angle) {
    m_arm = subsystem;
    m_target1 = arm1Angle;
    m_target2 = arm2Angle;

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
