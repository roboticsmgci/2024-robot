// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.PresetConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Inout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Presets {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command IntakePreset(Arm arm, Inout inout){
    return Commands.parallel(
      new ArmSet(arm, ()->PresetConstants.joint1Preset1, ()->PresetConstants.joint2Preset1),
      new IntakeSpeed(inout, 0.12)
    );
  }

  public static Command AmpPreset(Arm arm){
    return new ArmSet(arm, ()->1, ()->0);
  }

  public static Command SpeakerPreset(Arm arm, Inout inout, Pose3d shooterPos){
    return new ArmSet(arm, ()->1, ()->0);
  }

  public static Command TrapPreset(Arm arm){
    return new ArmSet(arm, ()->1, ()->0);
  }

  private Presets() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
