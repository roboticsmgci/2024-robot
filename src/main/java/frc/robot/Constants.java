// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
     * The deadband to apply to the controller.
     */
    public static final double kControllerDeadband = 0.05; // Can change this depending on controller being used

    /**
     * The multiplier for the default driving speed, between 0-1. 1 means maximum
     * speed.
     */
    public static final double kDefaultSpeed = 1;

    /**
     * The multiplier for the slower ddriving speed, between 0-1. 1 means maximum
     * speed.
     */
    public static final double kSlowSpeed = 0.1;
  }

  public static class DrivetrainConstants {

    /**
     * The maximum speed of the robot, in meters per second. This ideally should be
     * measured empirically.
     * <p>
     * Currently it's based on the free speed specified in the MK4i docs.
     */
    public static final double kMaximumSpeed = Units.feetToMeters(16.6);

    /**
     * The total mass of the robot, in kilograms.
     */
    public static final double kRobotMass = Units.lbsToKilograms(100);

    /**
     * A list of {@link Matter} objects that represent the robot chassis's various
     * parts.
     */
    public static final List<Matter> kChassis = List
        .of(new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass));

    /**
     * The time it takes for the velocity of the robot to be updated, in seconds.
     */
    public static final double kLoopTime = 0.13; // s, 20ms + 110ms sprk max velocity lag
  }

  public static class InoutConstants {
    /**
     * The ultrasonic sensor's minimum reading.
     */
    // public static final int kUltrasonicAnalogPort = 0;

    // /**
    //  * The ultrasonic sensor's range of output, in external units.
    //  */
    // public static final double kUltrasonicRange = 500;

    // /**
    //  * The starting point of the ultrasonic sensor, i.e., what the value is when the output is 0V.
    //  */
    // public static final double kUltrasonicOffset = 0;

    /**
     * The cutoff ultrasonic sensor value for whether there is a note in the intake.
     */
    public static final double kUltrasonicCutoff = 35;

    /**
     * The speed at which to run the intake motors to pick up a note.
     */
    public static final double kIntakeSpeed = 0.8;

    /**
     * The gear ratio of the intake.
     */
    public static final double kIntakeGearRatio = 1.0 / 1.0;

    /**
     * The gear ratio of the shooter.
     */
    public static final double kShooterGearRatio = 1.0 / 1.0;

    /**
     * The time (in milliseconds) to "warm up" the shooter before shooting.
     */
    public static final double kWarmupTime = 2000;

    /**
     * The speed at which to run the shooter.
     */
    public static final double kShooterSpeed = 1;


    public static final double kInitialShooterSpeed = 1;

    /**
     * The multiplier to use for the CIM speed of the top of the shooter (relative to the bottom NEO).
     */
    public static final double kCIMMultiplier = 1;

    public static final double kShooterTargetSpeed = 3000;
  }

  public final static class FieldConstants {
    /**
     * The x-coordinate of the center of the blue speaker.
     */
    //9
    public static final double kBlueSpeakerX = Units.inchesToMeters(0);

    /**
     * The y-coordinate of the center of the blue speaker.
     */
    //218.64
    public static final double kBlueSpeakerY = Units.inchesToMeters(0);

    /**
     * The x-coordinate of the center of the red speaker.
     */
    //642.25
    public static final double kRedSpeakerX = Units.inchesToMeters(0);

    /**
     * The y-coordinate of the center of the red speaker.
     */
    //218.64
    public static final double kRedSpeakerY = Units.inchesToMeters(0);

    
    public static final Pose2d[] kStartPoses = {
      new Pose2d(0.71, 6.68, new Rotation2d(Math.toRadians(60.00))),
      new Pose2d(1.36, 5.54, new Rotation2d()),
      new Pose2d(0.71, 4.40, new Rotation2d(Math.toRadians(-60.00))),
      new Pose2d(0.56, 3.09, new Rotation2d(Math.toRadians(-90.00)))
    };
    // TODO: add all notes
    public static final Pose2d[] kNotes = {
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(161.625), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(218.625), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(275.625), new Rotation2d()),
      //fill in, not currently used though
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(275.625), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(275.625), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(275.625), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(275.625), new Rotation2d()),
      new Pose2d(Units.inchesToMeters(120) - 0.321, Units.inchesToMeters(275.625), new Rotation2d())
    };

    public static final Pose2d[] kEndPoses = {
      new Pose2d(6, 6.68, new Rotation2d()),
      new Pose2d(5, 1.20, new Rotation2d())
    };

  }

  public static final class AutonConstants {

    public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants kAnglePID = new PIDConstants(0.4, 0, 0.01);
    public static final String[] kNotePaths = {
      "None",
      "1-1", "1-4", "1-5",
      "2-1", "2-2", "2-3",
      "3-3", "3-7", "3-7 (fast)", "3-8", "3-8 (fast)"
    };
  }

  public static final class PIDValues {
    public static final double kPLockTargetRotSpeed = 0.2;
    public static final double kILockTargetRotSpeed = 0;
    public static final double kDLockTargetRotSpeed = 0;
  }

   public static final class PresetConstants {
    
    // Intake (A)
    public static final double joint1Intake = 95.8;
    public static final double joint2Intake = -0.718;

    // Speaker (B)
    public static final double joint1Speaker = 62.0;
    public static final double joint2Speaker = -2.22;

    // Start
    public static final double joint1Initial = 1.57;
    public static final double joint2Initial = 0;

    public static final double joint1Amp = 3.1415;
    public static final double joint2Amp = -2.169;

    public static final double joint1Trap = 3.1415;
    public static final double joint2Trap = -2.169;
   }

  public static class CANConstants {
    public static final int kArmJoint1ID = 29;
    public static final int kArmJoint2ID = 21;
    public static final int kIntakeBottomID = 24;
    public static final int kIntakeTopID = 25;
    public static final int kShooterTopID = 22;
    public static final int kShooterBottomID = 30;
    public static final int kSPXID = 17;
  }

  public static final class ArmConstants {
    public static final double kArm1GearRatio = 1.0/16.0*32.0/16.0*64.0/32.0;
    public static final double kArm2GearRatio = 64.0/24.0*1.0/80.0;
    public static final double kShooterLength = Units.inchesToMeters(2);
    public static final double kArm2Length = Units.inchesToMeters(17);
    public static final Translation2d kArmBase = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(17.5));
    //redo
    public static final double kArm1Initial = Math.toRadians(90);
    public static final double kArm2Initial = Math.toRadians(0);
    public static final double kArm1MaxSpeed = 1.0;
    public static final double kArm2MaxSpeed = 1.0;

    public static final double kArm1RateLimit = 0.5;
  }
}
