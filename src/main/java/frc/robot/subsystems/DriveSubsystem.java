package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;

/**
 * Represents the drivetrain.
 */
public class DriveSubsystem extends SubsystemBase {

  // For Translation2D, note that positive x values represent moving toward the
  // front of the robot whereas positive y values represent moving toward the left
  // of the robot.

  /**
   * The location of the front left swerve module.
   */
  private final Translation2d m_frontLeftLocation = new Translation2d(DrivetrainConstants.kWheelBase / 2,
      DrivetrainConstants.kTrackWidth / 2);

  /**
   * The location of the front right swerve module.
   */
  private final Translation2d m_frontRightLocation = new Translation2d(DrivetrainConstants.kWheelBase / 2,
      -DrivetrainConstants.kTrackWidth / 2);

  /**
   * The location of the back left swerve module.
   */
  private final Translation2d m_backLeftLocation = new Translation2d(-DrivetrainConstants.kWheelBase / 2,
      DrivetrainConstants.kTrackWidth / 2);

  /**
   * The location of the back right swerve module.
   */
  private final Translation2d m_backRightLocation = new Translation2d(-DrivetrainConstants.kWheelBase / 2,
      -DrivetrainConstants.kTrackWidth / 2);

  /**
   * The swerve drive kinematics; used to convert chassis speeds (x speed, y
   * speed, rotational speed) to swerve module states and vice versa.
   */
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /**
   * The front left swerve module.
   */
  private final SwerveModule m_frontLeftModule = new SwerveModule(
      DrivetrainConstants.kFrontLeftDriveMotorID,
      DrivetrainConstants.kFrontLeftTurnMotorID,
      DrivetrainConstants.kFrontLeftCANcoderID,
      DrivetrainConstants.kFrontLeftDriveMotorInverted,
      DrivetrainConstants.kFrontLeftTurnMotorInverted,
      DrivetrainConstants.kFrontLeftTurnEncoderOffset);

  /**
   * The front right swerve module.
   */
  private final SwerveModule m_frontRightModule = new SwerveModule(
      DrivetrainConstants.kFrontRightDriveMotorID,
      DrivetrainConstants.kFrontRightTurnMotorID,
      DrivetrainConstants.kFrontRightCANcoderID,
      DrivetrainConstants.kFrontRightDriveMotorInverted,
      DrivetrainConstants.kFrontRightTurnMotorInverted,
      DrivetrainConstants.kFrontRightTurnEncoderOffset);

  /**
   * The back left swerve module.
   */
  private final SwerveModule m_backLeftModule = new SwerveModule(
      DrivetrainConstants.kBackLeftDriveMotorID,
      DrivetrainConstants.kBackLeftTurnMotorID,
      DrivetrainConstants.kBackLeftCANcoderID,
      DrivetrainConstants.kBackLeftDriveMotorInverted,
      DrivetrainConstants.kBackLeftTurnMotorInverted,
      DrivetrainConstants.kBackLeftTurnEncoderOffset);

  /**
   * The back right swerve module.
   */
  private final SwerveModule m_backRightModule = new SwerveModule(
      DrivetrainConstants.kBackRightDriveMotorID,
      DrivetrainConstants.kBackRightTurnMotorID,
      DrivetrainConstants.kBackRightCANcoderID,
      DrivetrainConstants.kBackRightDriveMotorInverted,
      DrivetrainConstants.kBackRightTurnMotorInverted,
      DrivetrainConstants.kBackRightTurnEncoderOffset);

  // TODO: not sure if specifying 100 is necessary
  /**
   * The gyro sensor.
   */
  private final AHRS m_gyro = new AHRS(Port.kMXP, (byte) 100);

  /**
   * The virtual field used for simulation.
   */
  private final Field2d m_field = new Field2d();

  /**
   * The pose estimator; used to estimate the current position of the robot on the
   * field.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
      },
      new Pose2d() // TODO: consider changing Pose2D based on chosen starting position
  ); // TODO: consider using SwerveDrivePoseEstimator with more constructors

  /**
   * Constructs a <code>DriveSubsystem</code>.
   */
  public DriveSubsystem() {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Updates odometry.
    m_poseEstimator.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    });
  }

  @Override
  public void simulationPeriodic() {
    // Calculates what the velocities of the robot should be based on the current
    // velocities and angles of the swerve modules.
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
        new SwerveModuleState[] {
            m_frontLeftModule.getState(),
            m_frontRightModule.getState(),
            m_backLeftModule.getState(),
            m_backRightModule.getState()
        });
    
    System.out.println("Actual: " + m_frontLeftModule.getState());

    // navX simulation stuff; the angle needs to be updated manually
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    double previousAngle = angle.get();
    // Convert to degrees, assuming 50 hz for simulationPeriodic (aka 20 ms
    // intervals)
    double changeInRotation = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond) / 50;
    // Minus sign because chassisSpeeds has increasing angles going CCW while gyro
    // has increasing angles going CW
    angle.set(previousAngle - changeInRotation);

    // Updates the robot's position on the virtual field.
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // TODO: potentially uncomment this to replace above line
    // NavX expects clockwise positive, but sim outputs clockwise negative
    // angle.set(Math.IEEEremainder(-drivetrainSim.getHeading().getDegrees(), 360));
  }

  /**
   * Gets the current heading of the robot between (-180, 180].
   * 
   * <pre>
   * {@code
   * Directions:
   *    0
   * 90   -90
   *   180
   * }
   * </pre>
   * 
   * @return the robot's heading
   */
  public double getRobotHeadingDegrees() {
    // Convert gyro angle to increase going CCW (instead of increase going CW) and
    // put between (-180, 180]
    return -Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  /**
   * Gets the current heading of the robot between (-180, 180], taking the field
   * oriented offset (i.e., the robot's starting angle) into consideration.
   * 
   * @return the robot's field heading
   */
  private double getFieldHeadingDegrees() {
    return -Math.IEEEremainder(m_gyro.getAngle() + DriverConstants.kFieldOrientedOffset, 360);
  }

  /**
   * Sets the robot motors to drive in the specified direction.
   * 
   * @param xSpeed        Forward velocity.
   * @param ySpeed        Sideways velocity.
   * @param rotationSpeed Angular velocity.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
    SwerveModuleState[] desiredSwerveModuleStates;

    if (DriverConstants.kIsFieldOriented) {
      desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
              Rotation2d.fromDegrees(getFieldHeadingDegrees())));
    } else {
      desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    }

    System.out.println("Desired: " + desiredSwerveModuleStates[0]);
    m_frontLeftModule.drive(desiredSwerveModuleStates[0]);
    m_frontRightModule.drive(desiredSwerveModuleStates[1]);
    m_backLeftModule.drive(desiredSwerveModuleStates[2]);
    m_backRightModule.drive(desiredSwerveModuleStates[3]);
  }

}