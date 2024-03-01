// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Inout;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class Auto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_swerve;
  private final Arm m_arm;
  private final Inout m_inout;

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
    
    //this.addCommands(create trajectory to startPos);
    System.out.println(startPos);
  }

  public void addNote(Pose2d notePos){
    //go to note, pick up note, shoot note
    //this.addCommands();
    System.out.println(notePos);
  }


  public void setEndPos(Pose2d endPos){
    //this.addCommands(create trajectory to endPos);
    System.out.println(endPos);
  }


}
