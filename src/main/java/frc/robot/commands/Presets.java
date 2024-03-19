// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Inout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Presets {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command IntakePreset(Arm arm, Inout inout){
    return Commands.parallel(
      new ArmSet(arm, ()->PresetConstants.joint1Intake, ()->PresetConstants.joint2Intake),
      new IntakeSpeed(inout, 0.3)
    );
  }

  public static Command AmpPreset(Arm arm){
    return new ArmSet(arm, () -> PresetConstants.joint1Amp, () -> PresetConstants.joint2Amp);
  }

  public static Command SpeakerPreset(Arm arm, Inout inout){
    return new ArmSet(arm, () -> PresetConstants.joint1Speaker, () -> PresetConstants.joint2Speaker);
  }

  public static Command AutoSpeakerPreset(Arm arm, Inout inout, Pose2d drivePose){
    return new ArmSet(arm, () -> PresetConstants.joint1Speaker, 
    ()->Math.PI-arm.getArmEncoder1()-
      RobotContainer.calcShooterAngle(drivePose, arm.getInoutPos(), 
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
          ? new Translation3d(FieldConstants.kRedSpeakerX, FieldConstants.kRedSpeakerY, FieldConstants.kSpeakerZ)
          : new Translation3d(FieldConstants.kBlueSpeakerX, FieldConstants.kBlueSpeakerY, FieldConstants.kSpeakerZ))
    );
  }

  public static Command TrapPreset(Arm arm){
    return new ArmSet(arm, ()->PresetConstants.joint1Trap, ()->PresetConstants.joint2Trap);
  }

  public static Command AutoTrapPreset(Arm arm, Inout inout, Pose2d drivePose){
    return new ArmSet(arm, () -> PresetConstants.joint1Trap, 
    ()->Math.PI-arm.getArmEncoder1()-
      RobotContainer.calcShooterAngle(drivePose, arm.getInoutPos(), 
      DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
      //find a trap location or nearest trap?
          ? new Translation3d(FieldConstants.kRedSpeakerX, FieldConstants.kRedSpeakerY, FieldConstants.kSpeakerZ)
          : new Translation3d(FieldConstants.kBlueSpeakerX, FieldConstants.kBlueSpeakerY, FieldConstants.kSpeakerZ))
    );
  }

  private Presets() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
