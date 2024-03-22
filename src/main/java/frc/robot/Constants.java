// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.util.PIDConstants;

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
    public static final int FORWARD = 0;
    public static final int RIGHT = 90;
    public static final int BACKWARD = 180;
    public static final int LEFT = 270;
    public static final int FORWARD_RIGHT = 45;
    public static final int BACKWARD_RIGHT = 135;
    public static final int BACKWARD_LEFT = 225;
    public static final int FORWARD_LEFT = 315;

    
    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0.0, 0.0001);
    }
    
    public static final class DrivebaseConstants {
        public static final double WHEEL_LOCK_TIME = 10; // seconds
        public static final double MAX_SPEED_FEET_PER_SECOND = 14.5;
    }
    
    public static final class Elevator {
        public static final int elevatorId = 12;
        
        public static final PIDFConfig pid = new PIDFConfig(0.8, 0.01, 0.05, 0);
        // public static final double kG = 0.04;
        public static final double kG = 0
        ;
        public static final double rangeSize = 0.2; //in inches
        public static final int rangeTime = 8; //in frames (runs at roughly 50 FPS)
        
        public static final double minPos = 0; //in ticks
        public static final double maxPos = 205; //in ticks
        public static final int currentLimit = 40;
        public static final double maxVBus = 0.9;
        
        public static final double tickToInchConversion = 27.0 / maxPos;
    }

    
    public static final class Shooter {
        public static final int leftId = 2;
        public static final int rightId = 1;
        
        public static final PIDFConfig pid = new PIDFConfig(5.5e-8, 7e-8, 0.0005, 0.000095/2);
        public static final double leftVelTarget = 6000;
        public static final double rightVelTarget = 6000;
        public static final double velDeadband = 750;
        public static final double velDeadbandTime = 12;
        
        public static final int maxCurrent = 25;
    }
    public static final class Indexer {
        public static final int leftId = 7;
        public static final int rightId = 4;
        public static final int photosensor1Id = 8;
        public static final int photosensor2Id = 9;
        
        public static final PIDConstants pid = new PIDConstants(4.0, 0.0, 1.0);
        
        public static final int maxCurrent = 25;
    }
    public static final class Intake {
        public static final int topId = 5;
        public static final int bottomId = 6;
        
        public static final PIDConstants pid = new PIDConstants(0.0002, 0.0000001, 0);
        
        public static final int maxCurrent = 25;
    }
    
    public static final class Pivot {
        public static final int pivotId = 3;
        
        public static final PIDFConfig pid = new PIDFConfig(0.12, 0, 0.03);
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kG = 0.03;
        public static final double toleranceDeg = 0.5;
        public static final double toleranceTime = 8;
        
        public static final double minPosTicks = -63.30;
        public static final double maxPosTicks = 1;
        public static final double maxVBus = 0.8;
        public static final double maxCurrent = 20;
        public static final boolean invertMotor = false;
        
        /*
        conversion math is (targetUnit1 - targetUnit2) / (currentUnit1 - currentUnit2). 1 and 2 are two reference positions to form a proportion
        FORMAT:
        public static final double <currentUnit>To<targetUnit>Conversion = ...;
        */
        public static final double ticksToDegConversion = (55.3 - 70 + 90) / (-1.67 - -34.78);
        public static final double relOffset = 0;
    }
    
    public static final class LEDConstants {
        public static final int ledId = 9;
        public static final int totalLength = 72; //in pixels
        public static final int headStart = 0;
        public static final int headEnd = totalLength / 4;
        public static final int visStart = totalLength / 4;
        public static final int visEnd = totalLength / 2;
    }
    
    public static class OperatorConstants {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
        public static final double TRIGGER_DEADBAND = 0.3;
        
        public static final int turnAxis = 4;
    }
    
    public static abstract class StateLocations {
        //elev is in inches, piv in degrees
        
        public static final double elevRest = 0; //done
        public static final double pivRest = -3 * Pivot.ticksToDegConversion;
        
        public static final double elevFloor = 0; //done
        public static final double pivFloor = -35.2 * Pivot.ticksToDegConversion;
        
        public static final double elevSource = 18.7; //done
        public static final double pivSource = -7.99 * Pivot.ticksToDegConversion;
        
        public static final double elevShootAmp = 27; //done
        public static final double pivShootAmp = (-30.58 + 8) * Pivot.ticksToDegConversion;
        
        public static final double elevShootSpeakerCenter = 15;
        public static final double pivShootSpeakerCenter = -64.32;
        public static final double elevShootSpeakerSide = 14;
        public static final double pivShootSpeakerSide = -64.32;

        public static final double elevClimb = 27;
        public static final double pivClimb = pivRest;
        
        public static final double safeElevatorPoint = 6; //the elevator hight required to turn the pivot freely and not hit anything
    }
    
    public static class ButtonBox {
        public static final int zero = 1;
        public static final int clear = 2;
        public static final int shoot = 3;
        public static final int reset = 4;
        public static final int speaker = 5;
        public static final int amp = 6;
        public static final int in = 7;
        public static final int rest = 8;
        public static final int source = 9;
        public static final int ground = 10;

        public static final int sliderAxis = 0;
        public static final double sliderUp = -0.69;
        public static final double sliderDown = 0.83;
        public static boolean getSliderUp(double sliderVal) {
            return sliderVal < sliderUp;
        }
        public static boolean getSliderDown(double sliderVal) {
            return sliderVal > sliderDown;
        }
    }

    public static class VisionConstants{
        //pipeline indexes - delete later
        public static final double APRILTAG_PIPELINE = 0.0;
        public static final double NEURAL_NETWORK_PIPELINE = 1.0;
        public static final double SPEAKER_LOCK_PIPELINE = 2.0; 
    
        //note tolerances
        public static final double TX_TOLERANCE_THRESHOLD = 5;  //degrees
        public static final double TARGET_DISTANCE_TOLERANCE_THRESHOLD = 0.1;
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

  public static class FixedLLConstants{
    public static final String FIXED_LL_NAME = "limelight-fixed";
    //pipelines
    public static final double FIXED_APRILTAG_PIPELINE = 0;
    public static final double FIXED_DRIVER_PIPELINE = 1;

    //default position of limelight in robotspace when in rest position (meters, degrees)
    public static final double DEFAULT_X_POSITION = -0.3556;
    public static final double DEFAULT_Y_POSITION = -0.1524;
    public static final double DEFAULT_Z_POSITION = 0.2667;
    public static final double DEFAULT_ROLL_POSITION = 5; //TBD
    public static final double DEFAULT_PITCH_POSITION = 35;
    public static final double DEFAULT_YAW_POSITON = 180;

  }
  public static class ShooterLLConstants{
    public static final String SHOOTER_LL_NAME = "limelight-shooter";
    //pipelines
    public static final double SHOOTER_APRILTAG_PIPELINE = 0;
    public static final double SHOOTER_NEURAL_NETWORK_PIPELINE = 1;
    public static final double SHOOTER_DRIVER_PIPELINE = 2;

    //default position of limelight in robotspace when in rest position
    public static final double DEFAULT_X_POSITION = 0.572;
    public static final double DEFAULT_Y_POSITION = 0.0;
    public static final double DEFAULT_Z_POSITION = 0.2667;
    public static final double DEFAULT_ROLL_POSITION = 0;
    public static final double DEFAULT_PITCH_POSITION = 350;  //TBD
    public static final double DEFAULT_YAW_POSITON = 0;
  }

  public static class ApriltagConstants{
    //Updating robot pose accuracy threshold
    public static final double BOTPOSE_THRESHOLD_TRANSLATION = Units.inchesToMeters(1); //meters
    public static final double BOTPOSE_THRESHOLD_ROTATION = 2;  //degrees

    //Offset for drive to tag-related commands 
    public static final double CAMERA_OFFSET = 30; //inches, needs tuning
    public static final double SUBWOOFER_OFFSET = 36.125; //inches, keep constant

    //Tag ids for each tags location
    public static final double BLUE_SOURCE_RIGHT_TAG = 1;
    public static final double BLUE_SOURCE_LEFT_TAG = 2;
    public static final double RED_SPEAKER_SIDE_TAG = 3 ;
    public static final double RED_SPEAKER_CENTER_TAG = 4;
    public static final double RED_AMP_TAG = 5;
    public static final double BLUE_AMP_TAG = 6;
    public static final double BLUE_SPEAKER_CENTER_TAG = 7;
    public static final double BLUE_SPEAKER_SIDE_TAG = 8;
    public static final double RED_SOURCE_RIGHT_TAG = 9;
    public static final double RED_SOURCE_LEFT_TAG = 10;
    public static final double RED_STAGE_SOURCESIDE_TAG = 11;
    public static final double RED_STAGE_AMPSIDE_TAG = 12;
    public static final double RED_STAGE_CENTERSIDE_TAG = 13;
    public static final double BLUE_STAGE_CENTERSIDE_TAG = 14;
    public static final double BLUE_STAGE_AMPSIDE_TAG = 15;
    public static final double BLUE_STAGE_SOURCE_TAG = 16;



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
        Rotation2d.fromDegrees(0)),

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
