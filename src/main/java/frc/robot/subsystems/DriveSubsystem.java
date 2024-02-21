package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import edu.wpi.first.wpilibj.DriverStation;
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
      Rotation2d.fromDegrees(getRobotHeadingDegrees()),
      new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_backLeftModule.getPosition(),
          m_backRightModule.getPosition()
      },
      new Pose2d() // TODO: consider changing Pose2D based on chosen starting position
  ); // TODO: consider using SwerveDrivePoseEstimator with more constructors

  /**
   * Whether the drive should be field oriented. <code>true</code> if it is field
   * oriented; <code>false</code> otherwise.
   */
  private boolean m_isFieldOriented = DriverConstants.kStartFieldOriented;

  /**
   * The gyro offset subtracted from the gyro's sensor actual reading.
   * <p>
   * This is used to "reset" the gyro by setting this value to the current gyro
   * value.
   */
  private double m_gyroOffset = -DriverConstants.kFieldOrientedOffset;

  /**
   * Constructs a <code>DriveSubsystem</code>.
   */
  public DriveSubsystem() {
    // Note: do not replace this with resetGyro()
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);

    // Setup PathPlanner
    AutoBuilder.configureHolonomic(
        m_poseEstimator::getEstimatedPosition,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            DrivetrainConstants.kMaxDriveMotorSpeed, // Max module speed, in m/s
            DrivetrainConstants.kTrackWidth, // Drive base radius in meters. Distance from robot center to furthest
                                             // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    // Updates odometry.
    m_poseEstimator.update(Rotation2d.fromDegrees(getRobotHeadingDegrees()), new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    });

    // Sys ID log stuff
    // m_frontLeftModule.logSysID();

    // TODO: uncomment this to find the turn encoder offsets to put in Constants
    // System.out.println("FL: " + m_frontLeftModule.getPosition().angle.getDegrees() +
    //                    ", FR: " + m_frontRightModule.getPosition().angle.getDegrees() +
    //                    ", BL: " + m_backLeftModule.getPosition().angle.getDegrees() +
    //                    ", BR: " + m_backRightModule.getPosition().angle.getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // Calculates what the velocities of the robot should be based on the current
    // velocities and angles of the swerve modules.
    ChassisSpeeds chassisSpeeds = getRobotRelativeSpeeds();

    // this is useful to test out the accuracy of feedforward
    // System.out.println("actual: " + m_frontLeftModule.getState());

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
    // TODO: potentially uncomment this to replace above line
    // NavX expects clockwise positive, but sim outputs clockwise negative
    // angle.set(Math.IEEEremainder(-drivetrainSim.getHeading().getDegrees(), 360));

    // Updates the robot's position on the virtual field.
    // TODO: consider moving this to periodic()
    m_field.setRobotPose(getPose());
    
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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
    return -Math.IEEEremainder(m_gyro.getAngle() - m_gyroOffset, 360);
  }

  /**
   * Sets the robot motors to drive in the specified direction.
   * 
   * @param xSpeed        Forward velocity, in meters per second.
   * @param ySpeed        Sideways velocity, in meters per second.
   * @param rotationSpeed Angular velocity, in radians per second.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
    SwerveModuleState[] desiredSwerveModuleStates;

    if (m_isFieldOriented) {
      desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed,
              Rotation2d.fromDegrees(getRobotHeadingDegrees())));
    } else {
      desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    }

    // this is useful to test out the accuracy of feedforward
    // System.out.println("desired: " + desiredSwerveModuleStates[0]);

    m_frontLeftModule.drive(desiredSwerveModuleStates[0]);
    m_frontRightModule.drive(desiredSwerveModuleStates[1]);
    m_backLeftModule.drive(desiredSwerveModuleStates[2]);
    m_backRightModule.drive(desiredSwerveModuleStates[3]);
  }

  /**
   * Sets the robot motors to drive in the specified robot-relative direction.
   * 
   * @param robotRelativeSpeeds the robot-relative <code>ChassisSpeeds</code>
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    SwerveModuleState[] desiredSwerveModuleStates = m_kinematics.toSwerveModuleStates(robotRelativeSpeeds);

    m_frontLeftModule.drive(desiredSwerveModuleStates[0]);
    m_frontRightModule.drive(desiredSwerveModuleStates[1]);
    m_backLeftModule.drive(desiredSwerveModuleStates[2]);
    m_backRightModule.drive(desiredSwerveModuleStates[3]);
  }

  /**
   * Resets the robot's pose to the given pose.
   * 
   * @param pose the given pose
   */
  private void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getRobotHeadingDegrees()), new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    }, pose);
    // TODO: not sure if this is necessary (it shouldn't be)
    // m_gyro.reset();
  }

  /**
   * Gets the speed of the robot, relative to the robot.
   * 
   * @return the robot relative <code>ChassisSpeeds</code>
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    // Calculates what the velocities of the robot should be based on the current
    // velocities and angles of the swerve modules.
    return m_kinematics.toChassisSpeeds(
        new SwerveModuleState[] {
            m_frontLeftModule.getState(),
            m_frontRightModule.getState(),
            m_backLeftModule.getState(),
            m_backRightModule.getState()
        });
  }

  /**
   * Returns whether the drive is field oriented.
   * 
   * @return <code>true</code> if the drive is field oriented; <code>false</code>
   *         if it is robot oriented.
   */
  public boolean getIsFieldOriented() {
    return m_isFieldOriented;
  }

  /**
   * Sets whether the drive is field oriented.
   * 
   * @param isFieldOriented <code>true</code> to set the drive to field oriented;
   *                        <code>false</code> to set it to robot oriented.
   */
  public void setIsFieldOriented(boolean isFieldOriented) {
    m_isFieldOriented = isFieldOriented;
  }

  /**
   * Resets the gyro sensor by updating the offset.
   * <p>
   * This is needed because the simulation uses the gyro sensor reading for the
   * rotation of the robot; resetting with m_gyro.reset() would cause the
   * simulated robot to rotate to be facing forwards.
   */
  public void resetGyro() {
    m_gyroOffset = m_gyro.getAngle();
  }

  public double targetHelper(double shooterSpeed, double shooterAngle, double targetAngle){
    double bestAngle = targetAngle;

    for(double i=-1800.0; i<1800; i++){
      //angle = i/10 ; 0.1 degree accuracy
      if(Math.abs(targetAngle-calculateTarget(shooterSpeed, shooterAngle, Math.toRadians(i/10)))<
          Math.abs(targetAngle-calculateTarget(shooterSpeed, shooterAngle, Math.toRadians(bestAngle)))){
        bestAngle = i/10;
      }
    }
    System.out.println(bestAngle);
    return bestAngle;
  }

  private double calculateTarget(double shooterSpeed, double shooterAngle, double robotAngle){
    ChassisSpeeds s = getRobotRelativeSpeeds();
    
    return Math.atan2(
      //Robot y velocity
      -s.vyMetersPerSecond*Math.sin(robotAngle)+s.vxMetersPerSecond*Math.cos(robotAngle)
      //Shooter launch y velocity
      +shooterSpeed*Math.cos(shooterAngle)*Math.cos(robotAngle)
      //Shooter rotational y velocity
      //-shooterradius*s.omegaRadiansPerSecond*Math.sin(robotAngle)
      , 
      //Robot x velocity
      -s.vyMetersPerSecond*Math.cos(robotAngle)-s.vxMetersPerSecond*Math.sin(robotAngle)
      //Shooter launch x velocity
      -shooterSpeed*Math.cos(shooterAngle)*Math.sin(robotAngle)
      //Shooter rotational x velocity
      //shooterradius*s.omegaRadiansPerSecond*Math.cos(robotAngle)
      );
      
  }

}