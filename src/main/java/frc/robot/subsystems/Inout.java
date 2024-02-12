// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Inout extends SubsystemBase {

  private final CANSparkMax intake;
  private final CANSparkMax shooter;
  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder shooterEncoder;
  
  private double shooterSpeed;

  public Inout() {
    intake = new CANSparkMax(9, MotorType.kBrushless);
    shooter = new CANSparkMax(10, MotorType.kBrushless);
    intakeEncoder = intake.getEncoder();
    intakeEncoder.setPositionConversionFactor(1*2*Math.PI);
    shooterEncoder = shooter.getEncoder();
    shooterEncoder.setVelocityConversionFactor(1*2*Math.PI);
  }

  public double getShooterSpeed(){
    return shooterSpeed;
  }

  public void setIntake(double speed){
    intake.set(speed);
  }

  public void setShooter(double speed){
    shooter.set(speed);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
