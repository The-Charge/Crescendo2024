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
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(4.0, 0.0, 1.0);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
  public static final class Elevator {
    public static final double setpoint = 2;
    public static final int leftId = 0;
    public static final int rightId = 1;
    public static final PIDFConfig elevatorPID   = new PIDFConfig(0.4, 0, 0.01);

    
  }

  public static final class Intake {
    public static final int topIntakeId = 0;
    public static final int bottomIntakeId = 1;
  }

  public static final class Shooter {
    public static final int leftShooterId = 0;
    public static final int rightShooterId = 1;
  }

  public static final class Indexer {
    public static final int leftIndexerId = 0;
    public static final int rightIndexerId = 1;
    public static final double limitSwitchIndexer = 2;
  }

  public static final class Pivot {
    public static final int PivotId = 7; 
    public static final double kFeedForward = 0;
    public static final PIDFConfig pivotPID = new PIDFConfig(0.7, 0, 0, kFeedForward);
    public static final double pivotkS = 0.25;
    public static final double pivotkV = 0.12;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;



  }

  public static final class Climber {
    public static final int climberId = 0;


  }

  public static final class LEDConstants {
    public static final int portId = 9;
    public static final int totalLength = 45; //in pixels
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class FieldConstants{
    //Fixed Apriltag poses as given by field drawings
    public static final Pose2d[] FIXED_APRILTAG_POSE = {
      new Pose2d(new Translation2d(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0)),
        Rotation2d.fromDegrees(0)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(593.68),
        Units.inchesToMeters(9.68)), 
        Rotation2d.fromDegrees(120)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(637.21),
        Units.inchesToMeters(34.79)), 
        Rotation2d.fromDegrees(120)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(652.73), 
        Units.inchesToMeters(196.17)), 
        Rotation2d.fromDegrees(180)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(652.73), 
        Units.inchesToMeters(218.42)), 
        Rotation2d.fromDegrees(180)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(578.77), 
        Units.inchesToMeters(323.00)), 
        Rotation2d.fromDegrees(270)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(72.5), 
        Units.inchesToMeters(323.00)), 
        Rotation2d.fromDegrees(270)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(-1.50), 
        Units.inchesToMeters(218.42)), 
        Rotation2d.fromDegrees(0)),
      
      new Pose2d(new Translation2d(
        Units.inchesToMeters(-1.50), 
        Units.inchesToMeters(196.17)), 
        Rotation2d.fromDegrees(0)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(14.02), 
        Units.inchesToMeters(34.79)), 
        Rotation2d.fromDegrees(60)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(57.54), 
        Units.inchesToMeters(9.68)), 
        Rotation2d.fromDegrees(60)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(468.69), 
        Units.inchesToMeters(146.19)), 
        Rotation2d.fromDegrees(300)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(468.69), 
        Units.inchesToMeters(177.10)), 
        Rotation2d.fromDegrees(60)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(441.74), 
        Units.inchesToMeters(161.62)), 
        Rotation2d.fromDegrees(180)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(209.48), 
        Units.inchesToMeters(161.62)), 
        Rotation2d.fromDegrees(0)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(182.73), 
        Units.inchesToMeters(177.10)), 
        Rotation2d.fromDegrees(120)),

      new Pose2d(new Translation2d(
        Units.inchesToMeters(182.73), 
        Units.inchesToMeters(146.19)), 
        Rotation2d.fromDegrees(240)),
    };
  }
  public static class ApriltagConstants{
    //Offset for drive to tag-related commands
    public static final double CAMERA_OFFSET = 30; 
    public static final double SUBWOOFER_OFFSET = 36.125;
    public static final Pose2d[] OFFSET_APRILTAG_POSE = {
      
    
      new Pose2d(new Translation2d(
        Units.inchesToMeters(0), 
        Units.inchesToMeters(0)), 
        Rotation2d.fromDegrees(0)),

      //Tag 1 - Blue Source RIGHT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(593.68 - CAMERA_OFFSET * Math.sin(Units.degreesToRadians(120))), 
        Units.inchesToMeters(9.68- CAMERA_OFFSET * Math.cos(Units.degreesToRadians(120)))), 
        Rotation2d.fromDegrees(120+180)),

      //Tag 2 - Blue Source LEFT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(637.21- CAMERA_OFFSET * Math.sin(Units.degreesToRadians(120))), 
        Units.inchesToMeters(34.79 - CAMERA_OFFSET * Math.cos(Units.degreesToRadians(120)))), 
        Rotation2d.fromDegrees(120+180)),

      //Tag 3 - Red Speaker SIDE
      new Pose2d(new Translation2d(
        Units.inchesToMeters(652.73 - SUBWOOFER_OFFSET - CAMERA_OFFSET), 
        Units.inchesToMeters(196.17)), 
        Rotation2d.fromDegrees(180+180)),

      //Tag 4 - Red Speaker CENTER
      new Pose2d(new Translation2d(
        Units.inchesToMeters(652.73 - SUBWOOFER_OFFSET - CAMERA_OFFSET), 
        Units.inchesToMeters(218.42)), 
        Rotation2d.fromDegrees(180+180)),

      //Tag 5 - Red Amp
      new Pose2d(new Translation2d(
        Units.inchesToMeters(578.77), 
        Units.inchesToMeters(323.00 - CAMERA_OFFSET)), 
        Rotation2d.fromDegrees(270+180)),

      //Tag 6 - Blue Amp
      new Pose2d(new Translation2d(
        Units.inchesToMeters(72.5), 
        Units.inchesToMeters(323.00 - CAMERA_OFFSET)), 
        Rotation2d.fromDegrees(270+180)),

      //Tag 7 - Blue Speaker CENTER 
      new Pose2d(new Translation2d(
        Units.inchesToMeters(-1.50 + SUBWOOFER_OFFSET + CAMERA_OFFSET), 
        Units.inchesToMeters(218.42)), 
        Rotation2d.fromDegrees(0+180)),

      //Tag 8 - Blue Speaker SIDE
      new Pose2d(new Translation2d(
        Units.inchesToMeters(-1.50 + SUBWOOFER_OFFSET + CAMERA_OFFSET), 
        Units.inchesToMeters(196.17)), 
        Rotation2d.fromDegrees(0+180)),

      //Tag 9 - Red Source RIGHT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(14.02 + CAMERA_OFFSET * Math.sin(Units.degreesToRadians(60))), 
        Units.inchesToMeters(34.79 + CAMERA_OFFSET * Math.cos(Units.degreesToRadians(60)))), 
        Rotation2d.fromDegrees(60+180)),

      //Tag 10 - Red Source LEFT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(57.54 + CAMERA_OFFSET * Math.sin(Units.degreesToRadians(60))), 
        Units.inchesToMeters(9.68 + CAMERA_OFFSET * Math.cos(Units.degreesToRadians(60)))), 
        Rotation2d.fromDegrees(60+180)),

      //Tag 11 - Red Stage BOTTOM RIGHT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(468.69 - CAMERA_OFFSET * Math.sin(Units.degreesToRadians(300))), 
        Units.inchesToMeters(146.19 - CAMERA_OFFSET * Math.cos(Units.degreesToRadians(300)))), 
        Rotation2d.fromDegrees(300+180)),

      //Tag 12 - Red Stage TOP RIGHT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(468.69 + CAMERA_OFFSET * Math.sin(Units.degreesToRadians(60))), 
        Units.inchesToMeters(177.10 + CAMERA_OFFSET * Math.cos(Units.degreesToRadians(60)))), 
        Rotation2d.fromDegrees(60+180)),

      //Tag 13 - Red Stage LEFT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(441.74 - CAMERA_OFFSET), 
        Units.inchesToMeters(161.62)), 
        Rotation2d.fromDegrees(180+180)),

      //Tag 14 - Blue Stage RIGHT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(209.48 + CAMERA_OFFSET), 
        Units.inchesToMeters(161.62)), 
        Rotation2d.fromDegrees(0+180)),

      //Tag 15 - Blue Stage TOP LEFT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(182.73 - CAMERA_OFFSET * Math.sin(Units.degreesToRadians(120))), 
        Units.inchesToMeters(177.10 - CAMERA_OFFSET * Math.cos(Units.degreesToRadians(120)))), 
        Rotation2d.fromDegrees(120+180)),

      //Tag 16 - Blue Stage BOTTOM LEFT
      new Pose2d(new Translation2d(
        Units.inchesToMeters(182.73 + CAMERA_OFFSET * Math.sin(Units.degreesToRadians(240))),
        Units.inchesToMeters(146.19 + CAMERA_OFFSET * Math.cos(Units.degreesToRadians(240)))), 
        Rotation2d.fromDegrees(240+180)),
    };
   
  }
}