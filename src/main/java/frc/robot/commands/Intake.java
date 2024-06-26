package frc.robot.commands;

import frc.robot.Constants.InoutConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Inout;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * IMPORTANT: doesn't work because no ultrasonic
 */
public class Intake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Inout m_inout;
  //private final DoubleSupplier m_speed, m_distance;

  public Intake(Inout subsystem) {
    m_inout = subsystem;
    //m_speed = speed;
    //m_distance = distance;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_inout.setIntake(InoutConstants.kIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  // When the note is in the correct location (pid?)
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  // @Os
}
