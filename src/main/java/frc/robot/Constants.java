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
     * The number of degrees CW the robot starts away from being "forwards".
     * <p>
     * For example, if the robot starts rotated 45 degrees CW, this should be equal
     * to 45. Note that this is overridden by the initial position specified in
     * PathPlanner.
     */
    public static final double kFieldOrientedOffset = 0;

    /**
     * Whether the robot's driving should start as field oriented. <code>true</code>
     * if it should start as field oriented; <code>false</code> if it should start
     * as robot oriented.
     */
    public static final boolean kStartFieldOriented = true;

    /**
     * The deadzone to apply to the controller.
     */
    public static final double kControllerDeadzone = 0.1;

    /**
     * The maximum horizontal speed to move the robot, in meters per second.
     * <p>
     * Used for the default swerve drive command.
     */
    public static final double kMaxHorizontalSpeed = 3;

    /**
     * The maximum rotational speed to move the robot, in radians per second.
     * <p>
     * Used for the default swerve drive command.
     */
    public static final double kMaxRotationalSpeed = 2.5;

    /**
     * The rate limit for the x and y control for the robot's drive.
     * <p>
     * Used for the slew rate limiter for the robot's drive.
     */
    public static final double kHorizontalRateLimit = 4;

    /**
     * The rate limit for the rotational control for the robot's drive.
     * <p>
     * Used for the slew rate limiter for the robot's drive.
     */
    public static final double kRotationalRateLimit = 6;
  }

  public static class DrivetrainConstants {
    // TODO: update dimensions
    // Dimensions in meters
    /**
     * The width of the robot, left to right, in meters.
     */
    public static final double kTrackWidth = Units.inchesToMeters(25.27559); // Distance left-right
    /**
     * The length of the robot, front to back, in meters.
     */
    public static final double kWheelBase = Units.inchesToMeters(25.27559); // Distance forwards-backwards
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

    /**
     * The turn gear ratio for the MK4i module. This is the same regardless of the
     * configuration (L1, L2, L3) being used.
     * <p>
     * Source: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
     */
    public static final double kTurnGearRatio = 1.0 / (150.0 / 7.0);

    // MK4i L1
    // public static final double kDriveGearRatio = 1.0 / ((14.0 / 50.0) * (25.0 /
    // 19.0) * (15.0 / 45.0));
    // MK4i L2
    // public static final double kDriveGearRatio = 1 / ((14.0 / 50.0) * (27.0 /
    // 17.0) * (15.0 / 45.0));
    // MK4i L3
    /**
     * The drive gear ratio for the MK4i module, using the L3 configuration.
     * <p>
     * Source: Source:
     * https://www.swervedrivespecialties.com/products/mk4i-swerve-module
     */
    public static final double kDriveGearRatio = 1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));

    // Multiply by degrees, divide by gear ratio
    /**
     * The number of degrees turned per revolution of the turn motor. This can be
     * used to convert the encoder value (which is in number of revolutions by
     * default) into degrees.
     * <p>
     * This is calculated by dividing 360 by the gear ratio.
     */
    public static final double kTurnDegreesPerEncoderRev = 360.0 / kTurnGearRatio;

    /**
     * The number of meters driven per revolution of the drive motor. This can be
     * used to convert the encoder value (which is in number of revolutions
     * by default) into meters.
     * <p>
     * This is calculated by dividing the wheel circumference by the gear ratio.
     */
    public static final double kDriveMetersPerEncoderRev = (kWheelDiameter * Math.PI) / kDriveGearRatio;

    /**
     * The stall torque of the turn motor in Newton-meters.
     * <p>
     * This value is for the NEO Brushless Motor V1.1. Source:
     * https://www.revrobotics.com/rev-21-1650/
     */
    public static final float kTurnMotorStallTorque = 2.6f;

    /**
     * The free speed of the turn motor in RPM.
     * <p>
     * This value is for the NEO Brushless Motor V1.1. Source:
     * https://www.revrobotics.com/rev-21-1650/
     */
    public static final int kTurnMotorFreeSpeed = 5676;

    /**
     * The stall torque of the drive motor in Newton-meters.
     * <p>
     * This value is for the NEO Brushless Motor V1.1. Source:
     * https://www.revrobotics.com/rev-21-1650/
     */
    public static final float kDriveMotorStallTorque = 2.6f;

    /**
     * The free speed of the drive motor in RPM.
     * <p>
     * This value is for the NEO Brushless Motor V1.1. Source:
     * https://www.revrobotics.com/rev-21-1650/
     */
    public static final int kDriveMotorFreeSpeed = 5676;

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
     * The maximum speed of a swerve module's drive motor in meters per second.
     * <p>
     * TODO: this must be determined experimentally. An underestimation is fine.
     */
    public static final double kMaxDriveMotorSpeed = 3.25;

  }

  public final static class PIDConstants {

    // public static final double kPModuleTurnPosition = .004;
    public static final double kPModuleTurnPosition = .000004;
    public static final double kIModuleTurnPosition = 0;
    public static final double kDModuleTurnPosition = 0;

    public static final double kPModuleDriveVelocity = .000004;
    public static final double kIModuleDriveVelocity = 0;
    public static final double kDModuleDriveVelocity = 0;

    public static final double kPLockTargetRotSpeed = .1;
    public static final double kILockTargetRotSpeed = 0;
    public static final double kDLockTargetRotSpeed = 0;

  }

  public final static class FieldConstants {
    /**
     * The x-coordinate of the center of the blue speaker.
     */
    public static final double kBlueSpeakerX = Units.inchesToMeters(9);

    /**
     * The y-coordinate of the center of the blue speaker.
     */
    public static final double kBlueSpeakerY = Units.inchesToMeters(218.64);

    /**
     * The x-coordinate of the center of the red speaker.
     */
    public static final double kRedSpeakerX = Units.inchesToMeters(642.25);

    /**
     * The y-coordinate of the center of the red speaker.
     */
    public static final double kRedSpeakerY = Units.inchesToMeters(218.64);

  }
}
