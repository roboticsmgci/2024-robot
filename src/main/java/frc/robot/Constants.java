// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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
    public static final double kTurnConstant = 6;
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
  }

  public static class DrivetrainConstants {

    /**
     * The maximum speed of the robot, in meters per second. This must be measured empirically.
     */
    public static final double kMaximumSpeed = 4.1 / 3.11 * 4;

    /**
     * The total mass of the robot, in kilograms.
     */
    public static final double kRobotMass = Units.lbsToKilograms(40);

    /**
     * A list of {@link Matter} objects that represent the robot chassis's various parts.
     */
    public static final List<Matter> kChassis = List.of(new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass));
    
    /**
     * The time it takes for the velocity of the robot to be updated, in seconds.
     */
    public static final double kLoopTime = 0.13; //s, 20ms + 110ms sprk max velocity lag

  }

  public final static class PIDValues {

    // public static final double kPModuleTurnPosition = .004;
    public static final double kPModuleTurnPosition = .004;
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

  public static final class AutonConstants {

    public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants kAnglePID = new PIDConstants(0.4, 0, 0.01);
  }
}
