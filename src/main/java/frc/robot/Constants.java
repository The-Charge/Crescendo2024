// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
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
    public static final int driverId = 12;
    
    public static final int currentLimit = 20;

    public static final PIDFConfig pid = new PIDFConfig(0.4, 0, 0.01, 0);
    public static final int magicCruisVelocity = 80;
    public static final int magicAcceleration = 160;
    public static final int magicJerk = 1600;
    public static final double rangeSize = 0.2; //0.2 rotations
    public static final int rangeTime = 20; //20 frames, ~0.4s

    public static final double minPos = 0;
    public static final double maxPos = 0;
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
    public static final int PivotId = 3;
    public static final double kFeedForward = 0;
    public static final PIDFConfig pivotPID = new PIDFConfig(0.7, 0, 0, kFeedForward);
    public static final double pivotkS = 0.25;
    public static final double pivotkV = 0.12;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int encoderId = 7;
  }

  public static final class Climber {
    public static final int climberId = 0;

  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    
    public static final double joystickDeadband = 0.1;
  }

  public static abstract class StateLocations {
    //elevator travel distance is 27 in

    public static final double elevStartup = 0;
    public static final double pivStartup = 155;
    public static final double elevPickupFloor = 0;
    public static final double pivPickupFloor = 20;
    public static final double elevPickupSource = 27 * 0.5;
    public static final double pivPickupSource = -45;
    public static final double elevShootAmp = 27 * 0.5;
    public static final double pivShootAmp = 45;
    public static final double elevHighRear = 27;
    public static final double pivHighRear = 80;
    public static final double elevShallowFront = 27 * 0.5;
    public static final double pivShallowFront = -45;
    public static final double elevSteepFront = 27 * 0.33;
    public static final double pivSteepFront = -20;
    public static final double elevTravel = 27 * 0.5;
    public static final double pivTravel = 0;

    public static final double elevTurnHight = 27 * 0.5; //the elevator hight required to turn the pivot freely and not hit anything
  }
}
