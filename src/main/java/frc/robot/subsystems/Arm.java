// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;

public class Arm extends SubsystemBase {
  // private final WPI_VictorSPX joint0 = new WPI_VictorSPX(CANConstants.kSPXID);
  private final CANSparkMax joint2 = new CANSparkMax(CANConstants.kArmJoint1ID, MotorType.kBrushless);
  private final CANSparkMax joint1 = new CANSparkMax(CANConstants.kArmJoint2ID, MotorType.kBrushless);
  
  //cim on victor
  private final RelativeEncoder encoder1 = joint1.getEncoder();
  private final RelativeEncoder encoder2 = joint2.getEncoder();
  
  // private final double arm2 = 1;
  /** Creates a new ExampleSubsystem. */
  public Arm() {
    // joint1.setIdleMode(IdleMode.kCoast);
    // joint2.setIdleMode(IdleMode.kCoast);

    // joint1.setInverted(true);
    // joint2.setInverted(true);

    // TODO
    joint1.setIdleMode(IdleMode.kBrake);
    joint1.setInverted(true);

    joint2.setIdleMode(IdleMode.kBrake);

    joint2.setSmartCurrentLimit(80);

    encoder1.setPositionConversionFactor(ArmConstants.kArm1GearRatio*2*Math.PI);
    encoder1.setPosition(ArmConstants.kArm1Initial * (-75.46458893280632));
    encoder2.setPositionConversionFactor(ArmConstants.kArm2GearRatio*2*Math.PI);
    encoder2.setPosition(ArmConstants.kArm2Initial * (-1.548524953789279 * (5.0 / 2)));
  }

  public void setArmEncoders(double arm1, double arm2){
    encoder1.setPosition(arm1 * (-75.46458893280632));
    encoder2.setPosition(arm2 * (-1.548524953789279 * (5.0 / 2)));
  }

  public Pose2d getArmPos(){
    return new Pose2d(ArmConstants.kArmBase.getX(), 
      ArmConstants.kArmBase.getY(), 
      new Rotation2d(getArmEncoder2()));
  }

  public Pose2d getInoutPos(){
    return new Pose2d(ArmConstants.kArmBase.getX()+ArmConstants.kArm2Length*Math.cos(getArmEncoder2()), 
      ArmConstants.kArmBase.getY()+ArmConstants.kArm2Length*Math.sin(getArmEncoder2()), 
      new Rotation2d(getArmEncoder1()+getArmEncoder2()));
  }

  public double getRelativeAngle(DoubleSupplier absoluteAngle){
    return -getArmEncoder1()+absoluteAngle.getAsDouble();
  }
  
  public void setArm1(double speed){
    double actualSpeed = MathUtil.clamp(speed, -1, 1);
    // System.out.println(Math.cos(getArmEncoder1()) + " " + (getArmEncoder1()+getArmEncoder2()));

    // TODO: uncomment this when we stop being a defence bot
    joint1.set((actualSpeed*ArmConstants.kArm1MaxSpeed) + ((-0.04) * Math.cos(getArmEncoder1())));

    // joint1.set(actualSpeed*ArmConstants.kArm1MaxSpeed);
  }

  //add gravity compensator like arm1
  public void setArm2(double speed){

    double actualSpeed = MathUtil.clamp(speed, -1, 1);

    // TODO: uncomment this when we stop being a defence bot

    if (getArmEncoder1() >= Math.toRadians(-10)) {
      joint2.set((actualSpeed*ArmConstants.kArm2MaxSpeed) + ((-0.14) * Math.cos(getArmEncoder1()+getArmEncoder2() + Math.toRadians(45))));
    } else {
      joint2.set(actualSpeed*ArmConstants.kArm2MaxSpeed);
    }

    

    // joint2.set(MathUtil.clamp(speed, -1, 1)*ArmConstants.kArm2MaxSpeed);
  }

  // public void setArm0(double speed){
  //   joint0.set(MathUtil.clamp(speed, -1, 1));
  // }

  public double getArmEncoder1() {
    return encoder1.getPosition() / (-75.46458893280632);
  }

  public double getArmEncoder2() {
    return encoder2.getPosition() / (-1.548524953789279 * (5.0 / 2));
  }

  public double getAbsoluteArmAngle() {
    return getArmEncoder1() + getArmEncoder2();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command drive(DoubleSupplier joint1, DoubleSupplier joint2) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          // set the speed of the arm joints
        });
  }

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
    SmartDashboard.putNumber("joint1", getArmEncoder1());
    SmartDashboard.putNumber("joint2", getArmEncoder2());
    // This method will be called once per scheduler run
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
                joint1.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            joint1.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(encoder1.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(encoder1.getVelocity(), MetersPerSecond));
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
