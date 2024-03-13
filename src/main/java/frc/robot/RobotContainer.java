// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.sql.Driver;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InoutConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.ArmSet;
import frc.robot.commands.Auto;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.InoutDrive;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.IntakeTime;
import frc.robot.commands.Shoot;
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
  private final SwerveSubsystem m_drive = new SwerveSubsystem();

  private final Arm m_arm = new Arm();
  // TODO: 1) add this back
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

  private final SendableChooser<Pose2d> m_startPosChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> m_endPosChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> m_note1Chooser = new SendableChooser<>();

  /**
   * An empty command, just used to give the auto chooser an option for the generated auto. DO NOT RUN THIS COMMAND.
   */
  private final Command m_dummyGeneratedAuto = Commands.none();

  private double crad = 0;

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
    configureChoosers();
    // Configure the trigger bindings
    configureBindings();

    // m_drive.resetOdometry(new Pose2d());
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // m_drive.setDefaultCommand(m_swerveDriveCommand);
    m_drive.setDefaultCommand(m_drive.driveCommand(
        () -> MathUtil.applyDeadband(m_controller.getAxis(1), DriverConstants.kControllerDeadband),
        () -> -MathUtil.applyDeadband(m_controller.getAxis(0), DriverConstants.kControllerDeadband),
        () -> -MathUtil.applyDeadband(m_controller.getAxis(4), DriverConstants.kControllerDeadband)));
    // TODO: add this back

   //checks if presets are being used, only active when button is pressed 
    

    m_arm.setDefaultCommand(new ArmSet(m_arm, () -> m_arm.getArmEncoder1(), () -> m_arm.getArmEncoder2()));

    // m_arm.setDefaultCommand(new ArmDrive(
    //   m_arm,
    //   () -> MathUtil.applyDeadband(m_armController.getLeftY(), 0.15) * 1,
    //   () -> MathUtil.applyDeadband(m_armController.getRightY(), 0.15) * 1));
    
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
    // m_driverController.x().onTrue(Commands.runOnce(()->m_drive.momentum=!m_drive.momentum));

    m_driverController.rightTrigger().onTrue(new ToggleFieldOriented(m_drive));
    // m_armController.a().onTrue((new IntakeTime(m_inout, 0.5, 0.8)).andThen(new IntakeTime(m_inout, 0.1, -0.8)));

    m_driverController.leftTrigger().onTrue(Commands.runOnce(m_drive::zeroGyro));
    // m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(Commands.runOnce(()->m_drive.resetGyro()));

    m_driverController.leftBumper()
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
    
    // TODO: 1) add back 2 lines
    m_armController.leftTrigger().onTrue(new IntakeSpeed(m_inout, 0.3)).onFalse(new IntakeTime(m_inout, 50, -0.1));
    m_armController.rightTrigger().whileTrue(new Shoot(m_inout, 0.2, 0));
    m_armController.rightStick().whileTrue(new Shoot(m_inout, 1, InoutConstants.kWarmupTime));

    m_armController.leftStick().and(m_armController.rightStick()).onTrue(Commands.runOnce(() -> m_arm.setArm(90, 0)));

    // TODO: add back
    m_armController.leftBumper().and(m_armController.rightBumper()).whileTrue(new ArmDrive(
      m_arm,
      () -> MathUtil.applyDeadband(m_armController.getLeftY(), 0.1),
      () -> MathUtil.applyDeadband(m_armController.getRightY(), 0.1)
    ));
    // DriverConstants
    // MathUtil.applyDeadband(crad, crad)

    // TODO: add back
    m_armController.leftBumper().and(m_armController.rightBumper()).whileTrue(new InoutDrive(
      m_inout,
      () -> m_armController.getHID().getPOV() == 0 ? -1 : (m_armController.getHID().getPOV() == 180 ? 1 : 0),
      () -> 0
    ));

    
    m_armController.a().whileTrue(new ArmSet(m_arm, () -> {return PresetConstants.joint1Intake;}, () -> {return PresetConstants.joint2Intake;})); 
    m_armController.b().whileTrue(new ArmSet(m_arm, () -> {return PresetConstants.joint1Speaker;}, () -> {return PresetConstants.joint2Speaker;}));
    m_armController.x().whileTrue(new ArmSet(m_arm, () -> {return PresetConstants.joint1Initial;}, () -> {return PresetConstants.joint2Initial;}));
    m_armController.y().whileTrue(new IntakeSpeed(m_inout, -0.3));

    // m_driverController.a().onTrue(Commands.runOnce(()->m_drive.setIsFieldOriented(!m_drive.getIsFieldOriented())));

    // TODO: remove this after sysid is done
    // m_driverController.a().whileTrue(m_drive.sysIdDriveMotorCommand());
    // m_driverController.b().whileTrue(m_drive.sysIdAngleMotorCommand());

    // m_driverController.leftBumper().and(m_driverController.rightBumper()).onTrue(new
    // ResetGyro(m_drive));
    
  }

  private void configureChoosers() {
    NamedCommands.registerCommand("Intake", Auto.intakeNote(m_arm, m_inout));
    for (int i = 0; i < FieldConstants.kStartPoses.length; i++) {
      m_startPosChooser.addOption("Start " + i, FieldConstants.kStartPoses[i]);
    }

    for (int i = 0; i < FieldConstants.kNotes.length; i++) {
      m_note1Chooser.addOption("1st Note - " + i, FieldConstants.kNotes[i]);
    }

    for (int i = 0; i < FieldConstants.kEndPoses.length; i++) {
      m_endPosChooser.addOption("End " + i, FieldConstants.kEndPoses[i]);
    }

    autoChooser.addOption("Generated Auto", m_dummyGeneratedAuto);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Start Pos Chooser", m_startPosChooser);
    SmartDashboard.putData("Note 1 Chooser", m_note1Chooser);
    SmartDashboard.putData("End Pos Chooser", m_endPosChooser);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem)

    if (autoChooser.getSelected().equals(m_dummyGeneratedAuto)) {
      Pose2d startPos = m_startPosChooser.getSelected();
      Pose2d note1Pos = m_note1Chooser.getSelected();
      Pose2d endPos = m_endPosChooser.getSelected();

      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red)) {
        startPos = mirrorPose(startPos);
        note1Pos = mirrorPose(note1Pos);
        endPos = mirrorPose(endPos);
      }

      Auto generatedAuto = new Auto(m_drive, m_arm, m_inout);
      //generatedAuto.setStartPos(startPos);
      //generatedAuto.addNote(note1Pos);
      generatedAuto.addNote("LEAVE BLUE");
      //generatedAuto.setEndPos(endPos);

      // System.out.println(startPos + " " + note1Pos + " " + endPos);

      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("2-2"));
    } else {
      return autoChooser.getSelected();
    }
  }

  private Pose2d mirrorPose(Pose2d pose) {
    return new Pose2d(Units.inchesToMeters(651.75) - pose.getX(), pose.getY(), pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
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
    // m_arm.simulationInit();
  }

  public void teleopInit() {
    m_drive.teleopInit();
  }

  public static double calcShooterAngle(Pose2d drivePose, Translation2d inoutPos){
    double shooterX = drivePose.getX()-Math.cos(drivePose.getRotation().getRadians())*inoutPos.getX();
    double shooterY = drivePose.getY()-Math.sin(drivePose.getRotation().getRadians())*inoutPos.getX();
    double shooterZ = inoutPos.getY();

    // TODO: fix
    return 0.0;

  }
}
