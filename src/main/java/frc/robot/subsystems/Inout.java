// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InoutConstants;
import frc.robot.Constants.CANConstants;

/**
 * The Inout subsystem. Includes the intake and shooter.
 */
public class Inout extends SubsystemBase {

  /**
   * The intake motor.
   */
  private final CANSparkMax m_intake = new CANSparkMax(CANConstants.kIntakeTopID, MotorType.kBrushless);

  private final CANSparkMax m_intakeBottom = new CANSparkMax(CANConstants.kIntakeBottomID, MotorType.kBrushless);

  /**
   * The shooter motor.
   */
  private final CANSparkMax m_shooter = new CANSparkMax(CANConstants.kShooterTopID, MotorType.kBrushless);

  private final CANSparkMax m_shooterBottom = new CANSparkMax(CANConstants.kShooterBottomID, MotorType.kBrushless);

  /**
   * The encoder of the intake motor.
   */
  private final RelativeEncoder m_intakeEncoder = m_intake.getEncoder();

  /**
   * The encoder of the shooter motor.
   */
  private final RelativeEncoder m_shooterEncoder = m_shooter.getEncoder();

  /**
   * The ultrasonic sensor.
   */
  // private final AnalogPotentiometer m_ultrasonic = new AnalogPotentiometer(
  //   InoutConstants.kUltrasonicAnalogPort,
  //   InoutConstants.kUltrasonicRange,
  //   InoutConstants.kUltrasonicOffset);
  
  private double shooterSpeed;

  public Inout() {

    m_intake.restoreFactoryDefaults();
    m_intakeBottom.restoreFactoryDefaults();
    m_shooter.restoreFactoryDefaults();
    m_shooterBottom.restoreFactoryDefaults();

    // m_shooterBottom.setInverted(true);

    m_intake.setIdleMode(IdleMode.kBrake);
    m_intakeBottom.setIdleMode(IdleMode.kBrake);
    m_shooter.setIdleMode(IdleMode.kCoast);
    m_shooterBottom.setIdleMode(IdleMode.kCoast);
    // -3415.680998 39.142824
    m_intake.setSmartCurrentLimit(65);
    m_intakeBottom.setSmartCurrentLimit(65);

    m_shooter.setSmartCurrentLimit(65);
    m_shooterBottom.setSmartCurrentLimit(65);

    m_shooterBottom.setInverted(true);
    
    m_intakeEncoder.setPositionConversionFactor(360 * InoutConstants.kIntakeGearRatio);
    m_shooterEncoder.setVelocityConversionFactor(InoutConstants.kShooterGearRatio);
    
  }

  public double getShooterSpeed(){
    //testing
    // return 3500;
    return m_shooterEncoder.getVelocity();
  }

  /**
   * Sets the speed of the intake motor.
   * 
   * @param speed speed between [-1, 1]
   */
  public void setIntake(double speed){
    m_intake.setVoltage(RobotController.getBatteryVoltage() * MathUtil.clamp(speed, -1, 1)/* * InoutConstants.kCIMMultiplier*/);
    m_intakeBottom.setVoltage(RobotController.getBatteryVoltage() * MathUtil.clamp(speed, -1, 1));
  }

  /**
   * Sets the speed of the shooter motor.
   * 
   * @param speed speed between [-1, 1]
   */
  public void setShooter(double speed){
    m_shooter.set(MathUtil.clamp(speed, -1, 1));
    m_shooterBottom.set(-MathUtil.clamp(speed, -1, 1));
    // System.out.println(m_shooterBottom.get());
  }

  /**
   * Checks whether there is a note in the intake.
   * 
   * @return <code>true</code> if there is a note; <code>false</code> if there isn't a note
   */
  // public boolean hasNote() {
  //   return m_ultrasonic.get() < InoutConstants.kUltrasonicCutoff;
  // }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public Command drive(DoubleSupplier joint1, DoubleSupplier joint2) {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return run(
  //       () -> {
  //         // set the speed of the arm joints
  //       });
  // }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: remove logging
    // SmartDashboard.putNumber("Ultrasonic", m_ultrasonic.get());
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_intake, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_shooter, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
