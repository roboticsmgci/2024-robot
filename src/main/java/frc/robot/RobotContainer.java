// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SlowDown;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem m_drive = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The PathPlanner auto command chooser.
   */
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("New Auto");

  /**
   * The default swerve drive command.
   */
  private final DefaultDrive m_swerveDriveCommand = new DefaultDrive(
      m_drive,
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX(),
      () -> -m_driverController.getRightX(),
      () -> false,
      () -> false,
      () -> false,
      () -> false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(m_swerveDriveCommand);

    // m_drive.setDefaultCommand(new LockToTarget(
    // m_drive,
    // new Translation2d(0, 0),
    // () -> -m_driverController.getLeftY(),
    // () -> -m_driverController.getLeftX()));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // m_driverController.x().onTrue(Commands.runOnce(()->m_drive.momentum=!m_drive.momentum));

    m_driverController.y().onTrue(new ToggleFieldOriented(m_drive));
    //m_driverController.a().onTrue(Commands.runOnce(()->m_drive.setIsFieldOriented(!m_drive.getIsFieldOriented())));

    // m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(new ResetGyro(m_drive));
    //m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(Commands.runOnce(()->m_drive.resetGyro()));

    m_driverController.leftTrigger().and(m_driverController.rightTrigger()).onTrue(new SlowDown(m_drive));
    // m_driverController.leftTrigger().and(m_driverController.rightTrigger()).onTrue(Commands.runOnce(()->SwerveDrive.slowFactor=0.5)).
    // onFalse(Commands.runOnce(()->SwerveDrive.slowFactor=0.5));
    //for toggle use conditionalcommand

    // Forces the robot to face a speaker while the right stick is pressed. 
    //(use down button since stick is easy to release accidentally)
    // m_driverController.a()
    //     .onTrue(Commands.runOnce(() -> m_swerveDriveCommand
    //         .setTarget(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    //             ? new Translation2d(FieldConstants.kRedSpeakerX, FieldConstants.kRedSpeakerY)
    //             : new Translation2d(FieldConstants.kBlueSpeakerX, FieldConstants.kBlueSpeakerY)),
    //         m_drive))
    //     .onFalse(Commands.runOnce(() -> m_swerveDriveCommand.setTarget(null), m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);

    return autoChooser.getSelected();
  }
}
