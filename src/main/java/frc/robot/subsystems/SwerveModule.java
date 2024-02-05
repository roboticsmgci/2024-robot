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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  // private final PIDController m_driveVelocityController = new PIDController(
  // PIDConstants.kPModuleDriveVelocity, PIDConstants.kIModuleDriveVelocity,
  // PIDConstants.kDModuleDriveVelocity);

  /**
   * The PID controller for the velocity of the drive motor.
   */
  private final PIDController m_driveVelocityPID = new PIDController(PIDConstants.kPModuleDriveVelocity,
      PIDConstants.kIModuleDriveVelocity, PIDConstants.kDModuleDriveVelocity);

  /**
   * The PID controller for the position of the turn motor.
   */
  private final PIDController m_turnPositionPID = new PIDController(
      PIDConstants.kPModuleTurnPosition, PIDConstants.kIModuleTurnPosition, PIDConstants.kDModuleTurnPosition);

  /**
   * The feedforward to use for the simulation.
   */
  private final SimpleMotorFeedforward m_simFeedforward = new SimpleMotorFeedforward(0, 0.26967587043, 0.00566318889);

  /**
   * The feedforward to use for the real robot.
   * 
   * TODO: update this with proper values
   */
  private final SimpleMotorFeedforward m_realFeedforward = new SimpleMotorFeedforward(0, 0.26967587043, 0.00566318889);

  /**
   * Constructs a <code>SwerveModule</code>.
   * 
   * @param driveMotorID       the device ID of the drive motor
   * @param turnMotorID        the device ID of the turn motor
   * @param cancoderID         the device ID of the CANcoder
   * @param driveMotorInverted <code>true</code> if the drive motor should be
   *                           inverted; <code>false</code> otherwise
   * @param turnMotorInverted  <code>true</code> if the turn motor should be
   *                           inverted; <code>false</code> otherwise
   * @param turnEncoderOffset  the value of the CANcoder when the wheel is facing
   *                           forward
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

    // Set idle mode to brake
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turnMotor.setIdleMode(IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    // NOTE: the following code was commented because the conversion factors are
    // buggy, at least in simulation. Please use the following methods instead:
    // getDriveEncoderPositionM, getDriveEncoderPositionMPS

    // The default unit of the encoders is RPM... need to convert this to meters and
    // meters per second for the drive encoder. This is done using the conversion
    // factors.
    // m_driveEncoder.setPositionConversionFactor(DrivetrainConstants.kDriveMetersPerEncoderRev);
    // m_driveEncoder.setVelocityConversionFactor(DrivetrainConstants.kDriveMetersPerEncoderRev
    // / 60);

    m_turnEncoder = m_turnMotor.getEncoder();

    // NOTE: the following code was commented because the conversion factors are
    // buggy, at least in simulation. Please use the following methods instead:
    // getTurnEncoderPositionD, getTurnEncoderPositionDPS

    // Need to convert RPM to degrees and degrees per second for the turn encoder
    // m_turnEncoder.setPositionConversionFactor(DrivetrainConstants.kTurnDegreesPerEncoderRev);
    // m_turnEncoder.setVelocityConversionFactor(DrivetrainConstants.kTurnDegreesPerEncoderRev
    // / 60);

    // Need to use continuous input for turn PID controller because -180 degrees =
    // 180 degrees
    m_turnPositionPID.enableContinuousInput(-180, 180);

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

  // private double m_lastVelocity;
  // private long m_lastTime = 0;
  // private double m_voltage = 0;
  // public void logSysID() {
  // long currentTime = System.currentTimeMillis();
  // double currentVelocity = getDriveEncoderVelocityMPS();

  // if (m_lastTime != 0) {
  // double acceleration = (currentVelocity - m_lastVelocity) / (currentTime -
  // m_lastTime) * 1000;
  // System.out.println(m_voltage + " " + currentVelocity + " " + acceleration);
  // }
  // m_lastTime = currentTime;
  // m_lastVelocity = currentVelocity;
  // // for example, for simulation, s = 0, v = 0.26967587043, a = 0.00566318889
  // }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /**
   * Returns the position of the drive motor's encoder in meters.
   * <p>
   * This is necessary to use instead of the conversion factor because it appears
   * the conversion factor doesn't always work properly, at least in simulation.
   * 
   * @return the encoder's position in meters
   */
  private double getDriveEncoderPositionM() {
    return m_driveEncoder.getPosition() * DrivetrainConstants.kDriveMetersPerEncoderRev;
  }

  /**
   * Returns the velocity of the drive motor's encoder in meters per second.
   * <p>
   * This is necessary to use instead of the conversion factor because it appears
   * the conversion factor doesn't always work properly, at least in simulation.
   * 
   * @return the encoder's velocity in meters per second
   */
  private double getDriveEncoderVelocityMPS() {
    return m_driveEncoder.getVelocity() * (DrivetrainConstants.kDriveMetersPerEncoderRev / 60);
  }

  /**
   * Returns the position of the turn motor's encoder in degrees.
   * <p>
   * This is necessary to use instead of the conversion factor because it appears
   * the conversion factor doesn't always work properly, at least in simulation.
   * 
   * @return the encoder's position in degrees
   */
  private double getTurnEncoderPositionD() {
    return m_turnEncoder.getPosition() * DrivetrainConstants.kTurnDegreesPerEncoderRev;
  }

  /**
   * Returns the velocity of the turn motor's encoder in degrees per second.
   * <p>
   * This is necessary to use instead of the conversion factor because it appears
   * the conversion factor doesn't always work properly, at least in simulation.
   * 
   * @return the encoder's velocity in degrees per second
   */
  private double getTurnEncoderVelocityDPS() {
    return m_turnEncoder.getVelocity() * (DrivetrainConstants.kTurnDegreesPerEncoderRev / 60);
  }

  /**
   * Initializes the relative turn encoder using the value from the CANcoder.
   */
  public void initTurnRelativeEncoder() {
    // Use absolute encoder only if running on the robot; this removes need to
    // simulate absolute encoder on the sim.
    if (RobotBase.isReal()) {
      // TODO: Not sure if this is necessary. Uncomment if necessary.
      // m_turnCANcoder.getAbsolutePosition().waitForUpdate(10);

      // Converts absolute encoder value to degrees then subtracts encoder offset then
      // converts back to revolutions.
      m_turnEncoder.setPosition(((m_turnCANcoder.getAbsolutePosition().getValue() * 360) - m_turnEncoderOffset)
          / DrivetrainConstants.kTurnDegreesPerEncoderRev);
    } else {
      // If using a simulation, just assumes that the wheels are facing forward.
      m_turnEncoder.setPosition(0);
    }
  }

  /**
   * Gets the position (position of the drive encoder and angle of the turn motor)
   * of the swerve module.
   * 
   * @return the swerve module's position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveEncoderPositionM(), Rotation2d.fromDegrees(getTurnEncoderPositionD()));
  }

  /**
   * Gets the state (velocity of the drive motor and angle of the turn motor) of
   * the swerve module.
   * 
   * @return the swerve module's state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveEncoderVelocityMPS(), Rotation2d.fromDegrees(getTurnEncoderPositionD()));
  }

  /**
   * Gets the angle the module is currently facing using the turn encoder.
   * 
   * @return the angle the module is facing, in degrees
   */
  public double getTurnAngle() {
    return getTurnEncoderPositionD();
  }

  /**
   * Returns the closest "equivalent" desired angle to currentAngle.
   * For example, if currentAngle is 20 and desiredAngle is 350, it returns -10
   * (which is equivalent to 350 deg).
   * This basically calculates the target of the PID controller for the turn
   * motor.
   * 
   * @param currentAngle the current encoder value of the turn encoder
   * @param desiredAngle the desired angle, usually calculated using
   *                     toSwerveModuleStates from kinematics
   * @return the equivalent angle
   */
  private double findNearestEquivalent(double currentAngle, double desiredAngle) {
    // TODO: optimize this; surely there's a better way than
    // incrementing/decrementing until finding the angle
    // idek how this works lol

    double lowerBound;
    double upperBound;
    double lowerOffset = currentAngle % 360;
    if (lowerOffset >= 0) {
      lowerBound = currentAngle - lowerOffset;
      upperBound = currentAngle + (360 - lowerOffset);
    } else {
      upperBound = currentAngle - lowerOffset;
      lowerBound = currentAngle - (360 + lowerOffset);
    }
    while (desiredAngle < lowerBound) {
      desiredAngle += 360;
    }
    while (desiredAngle > upperBound) {
      desiredAngle -= 360;
    }
    if (desiredAngle - currentAngle > 180) {
      desiredAngle -= 360;
    } else if (desiredAngle - currentAngle < -180) {
      desiredAngle += 360;
    }
    return desiredAngle;
  }

  /**
   * Find the closest module state to the current angle that's equivalent to the
   * desired state. This is because it's sometimes faster for a wheel to turn and
   * go backwards than to go forwards, and SwerveDriveKinematics always looks for
   * a way to achieve the given chassis speeds with drive motors going forward.
   * 
   * @param desiredState the state returned by SwerveDriveKinematics
   * @param currentAngle the current angle of the swerve module
   * @return the "optimized" (i.e., closer to current state) desired module state
   */
  private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = findNearestEquivalent(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle - 180) : (targetAngle + 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * Sets the desired state of the module, which is the used as the target of the
   * PID controllers.
   * 
   * @param desiredState desired state with drive speed and angle.
   */
  public void drive(SwerveModuleState desiredState) {
    SwerveModuleState optimizedDesiredState = optimize(desiredState, Rotation2d.fromDegrees(getTurnAngle()));
    // System.out.println(desiredState + " " + optimizedDesiredState);
    // can remove this if not using logsysid
    // m_voltage = RobotController.getBatteryVoltage() *
    // desiredState.speedMetersPerSecond / DrivetrainConstants.kMaxDriveSpeed;

    // m_driveMotor.setVoltage(
    // RobotController.getBatteryVoltage() * desiredState.speedMetersPerSecond /
    // DrivetrainConstants.kMaxDriveSpeed);
    if (RobotBase.isReal()) {
      m_driveMotor.setVoltage(m_realFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond)
          + m_driveVelocityPID.calculate(getDriveEncoderVelocityMPS(), optimizedDesiredState.speedMetersPerSecond));
    } else {
      m_driveMotor.setVoltage(m_simFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond)
          + m_driveVelocityPID.calculate(getDriveEncoderVelocityMPS(), optimizedDesiredState.speedMetersPerSecond));
    }
    m_turnMotor.setVoltage(RobotController.getBatteryVoltage()
        * m_turnPositionPID.calculate(getTurnAngle(), optimizedDesiredState.angle.getDegrees()));
    // m_turnMotor.setVoltage(0);
  }

}
