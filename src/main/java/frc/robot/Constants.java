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
        
        public static final PIDFConfig pid = new PIDFConfig(5.5e-8, 7e-8, 0.0005, 0.000095);
        public static final double leftVelTarget = 10200;
        public static final double rightVelTarget = 10200;
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
        
        public static final double elevSource = 15; //done
        public static final double pivSource = -7.99 * Pivot.ticksToDegConversion;
        
        public static final double elevShootAmp = 27; //done
        public static final double pivShootAmp = (-30.58 + 8) * Pivot.ticksToDegConversion;
        
        public static final double elevShootSpeaker = 27; //touching subwoofer done
       // public static final double pivShootSpeaker = (-28.52 + 1) * Pivot.ticksToDegConversion;
         public static final double pivShootSpeaker = -60;
        public static final double elevClimb = 27;
        public static final double pivClimb = pivRest;
        
        public static final double safeElevatorPoint = 6; //the elevator hight required to turn the pivot freely and not hit anything
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
        public static final int unused1 = 9;
        public static final int src = 10; //pickup source location
        public static final int amp = 11; //shoot amp location
        public static final int gnd = 12; //pickup ground location
        public static final int unused2 = 13;
        public static final int elevOverride = 14;
         //manual elevator control
        public static final int pivOverride = 15; //manual pivot control
    }
}
