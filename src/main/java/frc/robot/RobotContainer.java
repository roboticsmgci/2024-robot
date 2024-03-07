// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.ArmSet;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.IntakeTime;
import frc.robot.commands.ToggleFieldOriented;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Inout;
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

  private final Arm m_arm = new Arm();
  private final Inout m_inout = new Inout();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  // TODO: move this port to constants
  public final CommandXboxController m_armController = new CommandXboxController(
      1);
  private final Controller m_controller = new Controller(m_driverController);

  /**
   * The PathPlanner auto command chooser.
   */
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("New Auto");

  private double crad = 0;
  /**
   * The default swerve drive command.
   */
  private final DefaultDrive m_swerveDriveCommand = new DefaultDrive(
      m_drive,
      () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), DriverConstants.kControllerDeadband),
      () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), DriverConstants.kControllerDeadband),
      () -> -MathUtil.applyDeadband(m_driverController.getRightX(), DriverConstants.kControllerDeadband),
      () -> false,
      () -> false,
      () -> false,
      () -> false);

  public void log() {
    crad = Math.max(Math.hypot(m_controller.getRawAxis(0), m_controller.getRawAxis(1)), crad);
    SmartDashboard.putNumber("controller radius", crad);
    SmartDashboard.putNumber("pose x", m_drive.getPose().getX());
    SmartDashboard.putNumber("pose y", m_drive.getPose().getY());
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_drive.resetOdometry(new Pose2d());
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // m_drive.setDefaultCommand(m_swerveDriveCommand);
    m_drive.setDefaultCommand(m_drive.driveCommand(
        () -> MathUtil.applyDeadband(m_controller.getAxis(0), DriverConstants.kControllerDeadband),
        () -> MathUtil.applyDeadband(m_controller.getAxis(1), DriverConstants.kControllerDeadband),
        () -> -MathUtil.applyDeadband(m_controller.getAxis(4), DriverConstants.kControllerDeadband)));
    
    m_arm.setDefaultCommand(new ArmSet(
    m_arm,
    () -> {
        if (m_armController.getHID().getAButton()) return PresetConstants.joint1Preset1;
        else if (m_armController.getHID().getBButton()) return PresetConstants.joint1Preset2;
        else if (m_armController.getHID().getXButton()) return PresetConstants.joint1Preset3;
        else if (m_armController.getHID().getAButton()) return PresetConstants.joint1Preset4;
        else return m_arm.getArmEncoder1();
    },() -> {
      if (m_armController.getHID().getAButton()) return PresetConstants.joint2Preset1;
      else if (m_armController.getHID().getBButton()) return PresetConstants.joint2Preset2;
      else if (m_armController.getHID().getXButton()) return PresetConstants.joint2Preset3;
      else if (m_armController.getHID().getYButton()) return PresetConstants.joint2Preset4;
      else return m_arm.getArmEncoder2();
    }));
    // () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
    // DriverConstants.kControllerDeadzone),
    // () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
    // DriverConstants.kControllerDeadzone),
    // () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
    // DriverConstants.kControllerDeadzone)));

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
    // m_armController.a().onTrue((new IntakeTime(m_inout, 0.5, 0.8)).andThen(new IntakeTime(m_inout, 0.1, -0.8)));
    m_armController.leftTrigger().onTrue(new IntakeSpeed(m_inout, 0.8)).onFalse(new IntakeTime(m_inout, 100, -0.8));

    m_armController.leftBumper().and(m_armController.rightBumper()).whileTrue(new ArmDrive(
      m_arm,
      () -> m_armController.getLeftY() * DriverConstants.kArmJoint1Speed,
      () -> m_armController.getRightY() * DriverConstants.kArmJoint2Speed
    ));
    // m_driverController.a().onTrue(Commands.runOnce(()->m_drive.setIsFieldOriented(!m_drive.getIsFieldOriented())));

    // TODO: remove this after sysid is done
    // m_driverController.a().whileTrue(m_drive.sysIdDriveMotorCommand());
    // m_driverController.b().whileTrue(m_drive.sysIdAngleMotorCommand());

    // m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(new
    // ResetGyro(m_drive));
    m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(Commands.runOnce(m_drive::zeroGyro));
    // m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(Commands.runOnce(()->m_drive.resetGyro()));

    m_driverController.leftTrigger()
        .and(m_driverController.rightTrigger())
        .onTrue(Commands.runOnce(() -> m_drive.setSlowFactor(DriverConstants.kSlowSpeed)))
        .onFalse(Commands.runOnce(() -> m_drive.setSlowFactor(DriverConstants.kDefaultSpeed)));

    // Forces the robot to face a speaker while the right stick is pressed.
    // (use down button since stick is easy to release accidentally)
    m_controller.getButton(1)
        .onTrue(Commands.runOnce(() -> m_drive
            .setTarget(DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                    ? new Translation2d(FieldConstants.kRedSpeakerX, FieldConstants.kRedSpeakerY)
                    : new Translation2d(FieldConstants.kBlueSpeakerX, FieldConstants.kBlueSpeakerY)),
            m_drive))
        .onFalse(Commands.runOnce(() -> m_drive.setTarget(null), m_drive));
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

  /**
   * Update the controller errors if it hasn't been updated yet.
   */
  public void updateControllerErrors() {
    // System.out.println(P"hello");
    if (m_controller.controller.getRawAxis(0) != 0 && m_controller.errors[0] == 0) {
      // System.out.println("updated");
      for (int i = 0; i < m_controller.errors.length; i++) {
        m_controller.errors[i] = m_controller.controller.getRawAxis(i);
      }
    }
  }

  public void simulationInit() {
    m_arm.simulationInit();
  }
}
