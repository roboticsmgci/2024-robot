// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANConstants;

/**
 * The arm subsystem. Includes the two arm joints.
 */
public class Arm extends SubsystemBase {

  /**
   * The arm joint closer to the robot base.
   */
  private final CANSparkMax m_joint1 = new CANSparkMax(CANConstants.kArmJoint1ID, MotorType.kBrushless);

  /**
   * The arm joint that rotates the intake & shooter.
   */
  private final CANSparkMax m_joint2 = new CANSparkMax(CANConstants.kArmJoint2ID, MotorType.kBrushless);
  // private final CANSparkMax intake = new CANSparkMax(16, MotorType.kBrushless);

  /**
   * The encoder of arm joint 1.
   */
  private final RelativeEncoder m_encoder1 = m_joint1.getEncoder();

  /**
   * The encoder of arm joint 2.
   */
  private final RelativeEncoder m_encoder2 = m_joint2.getEncoder();
  // private final RelativeEncoder intakeEncoder = intake.getEncoder();
  
  private final Translation2d m_base = new Translation2d(0, 0);
  // private final double arm1 = 1;
  // private final double arm2 = 1;
  
  /**
   * Constructs a new Arm object.
   */
  public Arm() {
    m_encoder1.setPositionConversionFactor(1*2*Math.PI);
    m_encoder1.setPosition(0);
    m_encoder2.setPositionConversionFactor(1*2*Math.PI);
    m_encoder2.setPosition(0);
    // intakeEncoder.setPositionConversionFactor(1*2*Math.PI);
    // intakeEncoder.setPosition(0);
  }

  /**
   * Get the current position of the arm.
   * 
   * @return
   */
  public Pose2d getArmPos(){
    return new Pose2d(m_base.getX(), 
      m_base.getY(), 
      new Rotation2d(m_encoder1.getPosition()));
  }

  // public Pose2d getInoutPos(){
  //   return new Pose2d(base.getX()+arm1*Math.cos(encoder1.getPosition()), 
  //     base.getY()+arm1*Math.sin(encoder1.getPosition()), 
  //     new Rotation2d(encoder1.getPosition()+intakeEncoder.getPosition()));
  // }
  
  /**
   * Sets the motor of arm joint 1.
   * 
   * @param speed speed between [-1, 1]
   */
  public void setArm1(double speed){
    m_joint1.setVoltage(MathUtil.clamp(speed, -1, 1) * RobotController.getBatteryVoltage());
  }

  /**
   * Sets the motor of arm joint 2.
   * 
   * @param speed speed between [-1, 1]
   */
  public void setArm2(double speed){
    m_joint2.setVoltage(MathUtil.clamp(speed, -1, 1) * RobotController.getBatteryVoltage());
  }

  /**
   * Gets the current value of the encoder of joint 1.
   * 
   * @return the encoder value
   */
  public double getArmEncoder1() {
    return m_encoder1.getPosition();
  }

  /**
   * Gets the current value of the encoder of joint 2.
   * 
   * @return the encoder value
   */
  public double getArmEncoder2() {
    return m_encoder2.getPosition();
  }

  // public void setInout(double speed){
  //   intake.set(speed);
  // }

  /**
   * Command that drives the arm motors at the values given by two `DoubleSupplier`s.
   *
   * @return the drive command
   */

  /**
   * Command that drives the arm motors at the values given by two `DoubleSupplier`s.
   * 
   * @param joint1 a supplier that returns the speed of joint 1, between [-1, 1]
   * @param joint2 a supplier that returns the speed of joint 1, between [-1, 1]
   * @return the drive command
   */
  public Command drive(DoubleSupplier joint1, DoubleSupplier joint2) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setArm1(joint1.getAsDouble());
          setArm2(joint2.getAsDouble());
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_joint1, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_joint2, DCMotor.getNEO(1));
    // REVPhysicsSim.getInstance().addSparkMax(intake, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_joint1.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_joint1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_encoder1.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_encoder1.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

              public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
                return m_sysIdRoutine.quasistatic(direction);
              }
            
              public Command sysIdDynamic(SysIdRoutine.Direction direction) {
                return m_sysIdRoutine.dynamic(direction);
              } 
}
