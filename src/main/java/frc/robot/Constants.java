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
    public static final PIDConstants ANGLE_PID = new PIDConstants(4.0, 0.0, 1.0);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double MAX_SPEED_FEET_PER_SECOND = 14.5;
  }

  public static final class Elevator {
    public static final int elevatorId = 12;
    public static final int limitId = 5;
    
    public static final int currentLimit = 20;

    public static final PIDFConfig pid = new PIDFConfig(0.8, 0.01, 0.05, 0);
    public static final double kG = 0.04;
    public static final double rangeSize = 0.2; //in inches
    public static final int rangeTime = 20; //in frames (runs at roughly 50 FPS)
    
    public static final double minPos = 0; //in ticks
    public static final double maxPos = 123; //in ticks
    public static final double ticksPerInch = maxPos / 27.0;
  }

  public static final class Intake {
    public static final int topId = 5;
    public static final int bottomId = 6;
    public static final PIDConstants pid = new PIDConstants(0.0002, 0.0000001, 0);
  }

  public static final class Shooter {
    public static final int leftId = 2;
    public static final int rightId = 1;
    public static final PIDConstants pid = new PIDConstants(4.0, 0.0, 1.0);
  }

  public static final class Indexer {
    public static final int photosensor1Id = 8;
    public static final int photosensor2Id = 9;
    public static final int leftId = 7;
    public static final int rightId = 4;
    public static final PIDConstants pid = new PIDConstants(4.0, 0.0, 1.0);
  }

  public static final class Pivot {
    public static final int pivotId = 3;
    public static final double kF = 0;
    public static final PIDFConfig pid = new PIDFConfig(0.12, 0, 0.03);
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kG = 0.03;

    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int encoderId = 7;
    
    public static final double minPos = 0;
    public static final double maxPos = 0;
    public static final double gearRat = 1 / 100.0;
    public static final double ticksPerDeg = 2048 / gearRat / 360.0;
    public static final double absEncoderAngleOffset = 0;
    public static final double absTicksPerDeg = 2048 / 360.0;
    public static final double absStart = 0.93; //0.56
    public static final double absRatio = -1* (0.93-0.56)/(37.19-0);
  }

  public static final class Climber {
    public static final int climberId = 0;

  }

  public static final class LEDConstants {
    public static final int ledId = 6;
    public static final int totalLength = 32; //in pixels
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double TRIGGER_DEADBAND = 0.3;
  }

  public static abstract class StateLocations {
    //elev is in inches, piv in degrees

    public static final double elevStartup = 0; //done
    public static final double pivStartup = 0;
    public static final double elevPickupFloor = 0; //done
    public static final double pivPickupFloor = -35.2;
    public static final double elevPickupSource = 68.34/Elevator.ticksPerInch; //done
    public static final double pivPickupSource = -7.99;
    public static final double elevShootAmp = 27; //done
    public static final double pivShootAmp = -30.58;
    public static final double elevHighRear = 86.56/Elevator.ticksPerInch; //touching subwoofer done
    public static final double pivHighRear = -28.52;
    public static final double elevShallowFront = 27 * 0.5;
    public static final double pivShallowFront = -45;
    public static final double elevSteepFront = 27 * 0.33;
    public static final double pivSteepFront = -20;
    public static final double elevTravel = 27 * 0.5;
    public static final double pivTravel = 0;

    public static final double safeElevatorPoint = 27 * 0.5; //the elevator hight required to turn the pivot freely and not hit anything
  }

  public static abstract class ButtonBox {
    public static final int rest = 1; //startup location
    public static final int clear = 2; //collector reverse all
    public static final int zero = 3; //collector zero
    public static final int shoot = 4; //collector shoot
    public static final int shB = 5; //shoot high rear location
    public static final int reset = 6; //move elevator down then reset
    public static final int inS = 7; //collector intake source
    public static final int inG = 8; //collector intake ground
    public static final int unused3 = 9;
    public static final int src = 10; //pickup source location
    public static final int amp = 11; //shoot amp location
    public static final int gnd = 12; //pickup ground location
    public static final int unused4 = 13;
    public static final int elevOVerride = 14; //manual elevator control
    public static final int pivOverride = 15; //manual pivot control
  }
}
