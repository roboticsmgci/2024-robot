// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    /**
     * The port index on the Driver Station that the controller is plugged into.
     */
    public static final int kDriverControllerPort = 0;
  }

  public static class DriverConstants {
    /**
     * The number of degrees CCW the robot starts away from being "forwards".
     * <p>
     * For example, if the robot starts rotated 45 degrees CCW, this should be equal
     * to 45.
     */
    public static final double kFieldOrientedOffset = 0;

    /**
     * Whether the robot's driving should start as field oriented. <code>true</code>
     * if it should start as field oriented; <code>false</code> if it should start
     * as robot oriented.
     */
    public static final boolean kStartFieldOriented = true;
  }

  public static class DrivetrainConstants {
    // TODO: update dimensions
    // Dimensions in meters
    /**
     * The width of the robot, left to right, in meters.
     */
    public static final double kTrackWidth = Units.inchesToMeters(22); // Distance left-right
    /**
     * The length of the robot, front to back, in meters.
     */
    public static final double kWheelBase = Units.inchesToMeters(27); // Distance forwards-backwards
    /**
     * The diameter of the robot's wheels, in meters.
     */
    private static final double kWheelDiameter = Units.inchesToMeters(4);

    // TODO: potentially update current limits
    /**
     * The smart current limit of the turn motor, in Amps.
     */
    public static final int kTurnMotorSmartLimit = 20;

    /**
     * The smart current limit of the drive motor, in Amps.
     */
    public static final int kDriveMotorSmartLimit = 20;

    // TODO: potentially update volt compensation
    public static final double kVoltCompensation = 12.6;

    // TODO: potentially update value
    // Source for gear ratios:
    // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
    // Turn ratios are the same for the MK4i no matter if L1, L2, L3
    public static final double kTurnGearRatio = 1.0 / (150.0 / 7.0);
    // MK4i L1
    // public static final double kDriveGearRatio = 1.0 / ((14.0 / 50.0) * (25.0 /
    // 19.0) * (15.0 / 45.0));
    // MK4i L2
    // public static final double kDriveGearRatio = 1 / ((14.0 / 50.0) * (27.0 /
    // 17.0) * (15.0 / 45.0));
    // MK4i L3
    public static final double kDriveGearRatio = 1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));

    // Circumference divided by gear ratio
    public static final double kDriveMetersPerEncoderRev = (kWheelDiameter * Math.PI) / kDriveGearRatio;

    // Multiply by degrees, divide by gear ratio
    public static final double kTurnDegreesPerEncoderRev = 360.0 / kTurnGearRatio;

    // Source for these values: https://www.revrobotics.com/rev-21-1650/
    public static final float kTurnMotorStallTorque = 2.6f; // in Nm
    public static final int kTurnMotorFreeSpeed = 5676; // in RPM
    public static final float kDriveMotorStallTorque = 2.6f; // in Nm
    public static final int kDriveMotorFreeSpeed = 5676; // in RPM

    // TODO: update motor IDs
    public static final int kFrontLeftDriveMotorID = 4;
    public static final int kFrontLeftTurnMotorID = 5;
    public static final int kFrontLeftCANcoderID = 6;
    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontLeftTurnMotorInverted = false;
    public static final double kFrontLeftTurnEncoderOffset = 231.24; // TODO: should be the value of the encoder when
                                                                     // the wheel is facing forward

    public static final int kFrontRightDriveMotorID = 7;
    public static final int kFrontRightTurnMotorID = 8;
    public static final int kFrontRightCANcoderID = 9;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kFrontRightTurnMotorInverted = false;
    public static final double kFrontRightTurnEncoderOffset = 317.725;

    public static final int kBackLeftDriveMotorID = 10;
    public static final int kBackLeftTurnMotorID = 11;
    public static final int kBackLeftCANcoderID = 12;
    public static final boolean kBackLeftDriveMotorInverted = false;
    public static final boolean kBackLeftTurnMotorInverted = false;
    public static final double kBackLeftTurnEncoderOffset = 182.28;

    public static final int kBackRightDriveMotorID = 13;
    public static final int kBackRightTurnMotorID = 14;
    public static final int kBackRightCANcoderID = 15;
    public static final boolean kBackRightDriveMotorInverted = false;
    public static final boolean kBackRightTurnMotorInverted = false;
    public static final double kBackRightTurnEncoderOffset = 254.18;

    /**
     * The maximum drive speed in meters per second.
     * <p>
     * TODO: this must be determined experimentally
     */
    public static final double kMaxDriveSpeed = 3.25;

  }

  public final static class PIDConstants {

    // public static final double kPModuleTurnPosition = .004;
    public static final double kPModuleTurnPosition = .000004;
    public static final double kIModuleTurnPosition = 0;
    public static final double kDModuleTurnPosition = 0;

  }
}
