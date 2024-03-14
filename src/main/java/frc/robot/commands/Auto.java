// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InoutConstants;
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

  //Call as first command
  public void setDelayTime(double time){
    this.addCommands(new WaitCommand(time));
  }

  public void setStartPos(Pose2d startPos){
    m_startPos = startPos;
    //this.addCommands(create trajectory to startPos);
    this.addCommands(Commands.runOnce(() -> m_swerve.resetOdometry(startPos), m_swerve));
    //4_3 setup
    if(startPos == FieldConstants.kStartPoses[3]){
      this.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("4_3 setup")));
    }
    //preload
    this.addCommands(shootNote(m_arm, m_inout));
  }

  //use new version
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
        new ArmSet(m_arm, () -> PresetConstants.joint1Speaker, () -> PresetConstants.joint1Speaker)
      ));
    }

    this.addCommands(shootNote(m_arm, m_inout));
    // TODO: pick up note, shoot note
  }

  //use this one
  public void addNote(String path){
    //Lower arm while driving for far notes, or else lower arm first
    //1 note paths are formatted s-n (startposid-noteid)
    if(path.charAt(0)!='2'&&path.charAt(2)>=4){
      this.addCommands(
        Commands.parallel(
          //Intake is an event part of the path
          AutoBuilder.followPath(PathPlannerPath.fromPathFile(path)),
          setupIntake(m_arm, m_inout)
        ),
        shootNote(m_arm, m_inout)
      );
    }else{
    this.addCommands(
      setupIntake(m_arm, m_inout),
      AutoBuilder.followPath(PathPlannerPath.fromPathFile(path)),
      shootNote(m_arm, m_inout)
      );
    }
  }


  public void setEndPos(Pose2d endPos){
    this.addCommands(m_swerve.driveToPose(endPos));
  }

  //Intake a note
  public static Command intakeNote(Arm arm, Inout inout) {
    return Commands.parallel(
      new ArmSet(arm, () -> PresetConstants.joint1Intake, () -> PresetConstants.joint2Intake),
      new InoutDrive(inout, ()->0.3, ()->0)
    );
  }

  //Set the arm position and warm up shooter
  public static Command setupShot(Arm arm, Inout inout){
    ArmSet armSet = new ArmSet(arm, () -> PresetConstants.joint1Speaker, () -> PresetConstants.joint2Speaker);
    return Commands.parallel(
      armSet,
      new InoutDrive(inout, ()->0, ()->1)
    ).until(()->(armSet.atSetpoint()&&inout.getShooterSpeed()>=InoutConstants.kShooterTargetSpeed));
  }

  //Lower arm and stop shooter
  public static Command setupIntake(Arm arm, Inout inout){
    ArmSet armSet = new ArmSet(arm, () -> PresetConstants.joint1Intake, () -> PresetConstants.joint2Intake);
    return Commands.parallel(
      armSet,
      new InoutDrive(inout, ()->0, ()->0)
    ).until(()->armSet.atSetpoint());
  }

  //Shoot a note from the subwoofer
  public static Command shootNote(Arm arm, Inout inout) {
    return Commands.sequence(
      //Remainder of arm setup
      setupShot(arm, inout),
      //Shoot for 1 second while holding shooter/arm
      Commands.deadline(
        new WaitCommand(1),
        new InoutDrive(inout, ()->1, ()->1),
        new ArmSet(arm, () -> PresetConstants.joint1Speaker, () -> PresetConstants.joint2Speaker)
      )
    );
  }


}
