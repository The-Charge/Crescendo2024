// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(4.0, 0.0, 1.0);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.01;
    public static final double LEFT_Y_DEADBAND  = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ApriltagConstants{
    //Apriltag poses relative to bottom left corner of field
    //0 is left empty for easy row-referencing in code
    public static final Pose2d[] APRILTAG_POSE = {
      new Pose2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68)), Rotation2d.fromDegrees(120)),
      new Pose2d(new Translation2d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79)), Rotation2d.fromDegrees(120)),
      new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17)), Rotation2d.fromDegrees(180)),
      new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), Rotation2d.fromDegrees(180)),
      new Pose2d(new Translation2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00)), Rotation2d.fromDegrees(270)),
      new Pose2d(new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00)), Rotation2d.fromDegrees(270)),
      new Pose2d(new Translation2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42)), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17)), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79)), Rotation2d.fromDegrees(60)),
      new Pose2d(new Translation2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68)), Rotation2d.fromDegrees(60)),
      new Pose2d(new Translation2d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19)), Rotation2d.fromDegrees(300)),
      new Pose2d(new Translation2d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10)), Rotation2d.fromDegrees(60)),
      new Pose2d(new Translation2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62)), Rotation2d.fromDegrees(180)),
      new Pose2d(new Translation2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62)), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10)), Rotation2d.fromDegrees(120)),
      new Pose2d(new Translation2d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19)), Rotation2d.fromDegrees(240)),
    };
  }
}
