// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {

  private final CANSparkMax joint1;
  private final CANSparkMax joint2;
  private final RelativeEncoder encoder1;
  private final RelativeEncoder encoder2;
  
  private final Translation2d base = new Translation2d(0, 0);
  private final double arm1 = 1;
  // private final double arm2 = 1;
  /** Creates a new ExampleSubsystem. */
  public Arm() {
    joint1 = new CANSparkMax(9, MotorType.kBrushless);
    joint2 = new CANSparkMax(10, MotorType.kBrushless);
    encoder1 = joint1.getEncoder();
    encoder1.setPositionConversionFactor(1*2*Math.PI);
    encoder1.setPosition(0);
    encoder2 = joint2.getEncoder();
    encoder2.setPositionConversionFactor(1*2*Math.PI);
    encoder2.setPosition(0);
  }

  public Pose2d getArmPos(){
    return new Pose2d(base.getX(), 
      base.getY(), 
      new Rotation2d(encoder1.getPosition()));
  }

  public Pose2d getInoutPos(){
    return new Pose2d(base.getX()+arm1*Math.cos(encoder1.getPosition()), 
      base.getY()+arm1*Math.sin(encoder1.getPosition()), 
      new Rotation2d(encoder1.getPosition()+encoder2.getPosition()));
  }

  public void setArm(double speed){
    joint1.set(speed);
  }

  public void setInout(double speed){
    joint2.set(speed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
