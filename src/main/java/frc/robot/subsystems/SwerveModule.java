package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PIDConstants;

/**
 * Represents a swerve module.
 */
public class SwerveModule extends SubsystemBase {

  /**
   * The module's drive motor.
   */
  public final CANSparkMax m_driveMotor;

  /**
   * The module's turn motor.
   */
  public final CANSparkMax m_turnMotor;

  /**
   * The encoder of the module's drive motor.
   */
  public final RelativeEncoder m_driveEncoder;

  /**
   * The encoder of the module's turn motor.
   */
  public final RelativeEncoder m_turnEncoder;

  /**
   * The absolute encoder (CANcoder) of the module.
   */
  public final CANcoder m_turnCANcoder;

  /**
   * The value of the CANcoder when the wheel is facing forward; this is
   * subtracted from the absolute encoder value to determine the initial relative
   * encoder value.
   */
  public final double m_turnEncoderOffset;

  /**
   * The PID controller for the velocity of the drive motor.
   */
  private final PIDController m_driveVelocityController = new PIDController(
      PIDConstants.kPModuleDriveVelocity, PIDConstants.kIModuleDriveVelocity, PIDConstants.kDModuleDriveVelocity);

  /**
   * The PID controller for the position of the turn motor.
   */
  private final PIDController m_turnPositionController = new PIDController(
      PIDConstants.kPModuleTurnPosition, PIDConstants.kIModuleTurnPosition, PIDConstants.kDModuleTurnPosition);

  /**
   * Constructs a <code>SwerveModule</code>.
   * 
   * @param driveMotorID       the device ID of the drive motor
   * @param turnMotorID        the device ID of the turn motor
   * @param cancoderID         the device ID of the CANcoder
   * @param driveMotorInverted <code>true</code> if 
   * @param turnMotorInverted
   * @param turnEncoderOffset
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, boolean driveMotorInverted,
      boolean turnMotorInverted, double turnEncoderOffset) {
    // Set up motors
    // TODO: configure Alternate Encoder Mode if brushless maybe?
    // https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turnMotor.restoreFactoryDefaults();

    m_driveMotor.setSmartCurrentLimit(DrivetrainConstants.kDriveMotorSmartLimit);
    m_turnMotor.setSmartCurrentLimit(DrivetrainConstants.kTurnMotorSmartLimit);

    m_driveMotor.enableVoltageCompensation(DrivetrainConstants.kVoltCompensation);
    m_turnMotor.enableVoltageCompensation(DrivetrainConstants.kVoltCompensation);

    m_turnCANcoder = new CANcoder(cancoderID);
    // TODO: potentially change this back to old library
    MagnetSensorConfigs sensorConfig = new MagnetSensorConfigs();
    sensorConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; // Set range to [0, 1)
    m_turnCANcoder.getConfigurator().apply(sensorConfig);

    m_turnEncoderOffset = turnEncoderOffset;

    m_driveMotor.setInverted(driveMotorInverted);
    m_turnMotor.setInverted(turnMotorInverted);

    // Set idle mode to break
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turnMotor.setIdleMode(IdleMode.kBrake);

    // The default unit of the encoders is RPM... need to convert this to meters and
    // meters per second for the drive encoder
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(DrivetrainConstants.kDriveMetersPerEncoderRev);
    m_driveEncoder.setVelocityConversionFactor(DrivetrainConstants.kDriveMetersPerEncoderRev / 60);

    // Need to convert RPM to degrees and degrees per second for the turn encoder
    m_turnEncoder = m_turnMotor.getEncoder();
    m_turnEncoder.setPositionConversionFactor(DrivetrainConstants.kTurnDegreesPerEncoderRev);
    m_turnEncoder.setVelocityConversionFactor(DrivetrainConstants.kTurnDegreesPerEncoderRev / 60);

    // Need to use continuous input for turn PID controller because -180 degrees =
    // 180 degrees
    m_turnPositionController.enableContinuousInput(-180, 180);

    // TODO: I don't think this is necessary, but in case data from the motor is not
    // working properly, uncomment this
    // m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    // m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    // m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

    // TODO: consider moving this to simulationInit in Robot.java
    if (RobotBase.isSimulation()) {
      // Source for stall torque and free speed:
      // https://www.revrobotics.com/rev-21-1650/
      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DrivetrainConstants.kDriveMotorStallTorque,
          DrivetrainConstants.kDriveMotorFreeSpeed);
      REVPhysicsSim.getInstance().addSparkMax(m_turnMotor, DrivetrainConstants.kTurnMotorStallTorque,
          DrivetrainConstants.kTurnMotorFreeSpeed);

      // TODO: consider uncommenting
      // m_driveEncoder.setPositionConversionFactor(1);
      // m_driveEncoder.setVelocityConversionFactor(1);

      // TODO: consider uncommenting
      // m_turnEncoder.setPositionConversionFactor(1);
      // m_turnEncoder.setVelocityConversionFactor(1);

      // m_turnPositionController.setP(.01);
      // m_driveVelocityController.setP(.25);

    }
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  // Use the absolute encoder to set the initial value of the relative encoder
  public void initTurnRelativeEncoder() {
    // Use absolute encoder only if running on the robot; this removes need to
    // simulate absolute encoder on the sim.
    if (RobotBase.isReal()) {
      // TODO: Not sure if this is necessary. Uncomment if necessary.
      // m_turnCANcoder.getAbsolutePosition().waitForUpdate(10);

      // Convert absolute encoder value to degrees then add encoder offset
      m_turnEncoder.setPosition((m_turnCANcoder.getAbsolutePosition().getValue() * 360) - m_turnEncoderOffset);
    } else {
      m_turnEncoder.setPosition(0);
    }
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromDegrees(m_turnEncoder.getPosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(m_turnEncoder.getPosition()));
  }

  public double getTurnAngle() {
    return m_turnEncoder.getPosition();
  }

  // TODO: is this necessary? equivalent of placeInAppropriate0To360Scope
  /**
   * Returns the closest "equivalent" desired angle to currentAngle.
   * For example, if currentAngle is 20 and desiredAngle is 350, it returns -10
   * (which is equivalent to 350 deg).
   * This makes it easier to do PID stuff as the target
   * 
   * @param currentAngle the current encoder value of the turn encoder
   * @param desiredAngle the desired angle, usually calculated using
   *                     toSwerveModuleStates from kinematics
   * @return the equivalent angle
   */
  // private double findNearestEquivalent(double currentAngle, double
  // desiredAngle) {

  // }

  // private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d
  // currentAngle) {

  // }

  /**
   * Sets the desired state of the module, which is the target of the PID
   * controllers.
   * 
   * @param desiredState Desired state with drive speed and angle.
   */
  public void drive(SwerveModuleState desiredState) {
    // TODO: make this actually complex
    m_driveMotor.setVoltage(
        RobotController.getBatteryVoltage() * desiredState.speedMetersPerSecond / DrivetrainConstants.kMaxDriveSpeed);
    m_turnMotor.setVoltage(RobotController.getBatteryVoltage()
        * m_turnPositionController.calculate(getTurnAngle(), desiredState.angle.getDegrees()) / 1000);
    // m_turnMotor.setVoltage(0);
  }

}
