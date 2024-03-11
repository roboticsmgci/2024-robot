// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Inout;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class Auto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_swerve;
  private final Arm m_arm;
  private final Inout m_inout;
  private Pose2d m_startPos;

  // private Pose2d m_startPos;
  // private ArrayList<Pose2d> m_notes = new ArrayList<>();
  // private Pose2d m_endPos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Auto(SwerveSubsystem swerve, Arm arm, Inout inout) {
    m_swerve = swerve;
    m_arm = arm;
    m_inout = inout;

  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, arm, inout);
  }

  public void setStartPos(Pose2d startPos){
    m_startPos = startPos;
    //this.addCommands(create trajectory to startPos);
    this.addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(startPos), m_swerve));
  }

  public void addNote(Pose2d notePos){
    //go to note, pick up note, shoot note
    //this.addCommands();
    // this.addCommands(m_swerve.driveToPose(notePos));
    this.addCommands(
      Commands.deadline(
        m_swerve.driveToPose(notePos), intakeNote(m_arm, m_inout)
      )
    );

    if (notePos == null) {
      this.addCommands(m_swerve.driveToPose(m_startPos));
    } else {
      this.addCommands(Commands.deadline(
        m_swerve.driveToPose(m_startPos),
        new ArmSet(m_arm, () -> PresetConstants.joint1Preset2, () -> PresetConstants.joint1Preset2)
      ));
    }

    this.addCommands(shootNote(m_arm, m_inout));
    // TODO: pick up note, shoot note
  }


  public void setEndPos(Pose2d endPos){
    //this.addCommands(create trajectory to endPos);
    this.addCommands(m_swerve.driveToPose(endPos));
  }

  public static Command intakeNote(Arm arm, Inout inout) {
    return Commands.parallel(
      new ArmSet(arm, () -> PresetConstants.joint1Preset1, () -> PresetConstants.joint2Preset1),
      new IntakeTime(inout, 0.12, 500)
    );
  }

  public static Command shootNote(Arm arm, Inout inout) {
    return Commands.parallel(
      new ArmSet(arm, () -> PresetConstants.joint1Preset2, () -> PresetConstants.joint2Preset2),
      Commands.deadline(
        new Shoot(inout, 1, 1000),
        new ArmSet(arm, () -> PresetConstants.joint1Preset1, () -> PresetConstants.joint2Preset1)
      )
    );
  }


}
