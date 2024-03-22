// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.*;
import frc.robot.commands.Elevator.MoveToSetpoint;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.*;

import javax.naming.spi.StateFactory;

import frc.robot.Constants;
import frc.robot.Constants.StateLocations;

import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class TeleopDrive extends Command {
    
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier heading;
    private final DoubleSupplier POV;
    private final BooleanSupplier shiftHalf;
    private final BooleanSupplier shiftQuarter;
    private final BooleanSupplier centricToggle;
    private double rotationSpeed;
    private boolean usePOV;
    private boolean isFieldCentric = true;
    private double allianceCorrection = 1;
    
    /**
    * Used to drive a swerve robot in full field-centric mode. vX and vY supply
    * translation inputs, where x is
    * torwards/away from alliance wall and y is left/right. headingHorzontal and
    * headingVertical are the Cartesian
    * coordinates from which the robot's angle will be derived— they will be
    * converted to a polar angle, which the robot
    * will rotate to.
    *
    * @param swerve  The swerve drivebase subsystem.
    * @param vX      DoubleSupplier that supplies the x-translation joystick input.
    *                Should be in the range -1 to 1 with
    *                deadband already accounted for. Positive X is away from the
    *                alliance wall.
    * @param vY      DoubleSupplier that supplies the y-translation joystick input.
    *                Should be in the range -1 to 1 with
    *                deadband already accounted for. Positive Y is towards the left
    *                wall when looking through the driver
    *                station glass.
    * @param heading DoubleSupplier that supplies the robot's heading angle.
    */
    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
    DoubleSupplier heading, DoubleSupplier POV, BooleanSupplier shiftHalf, BooleanSupplier shiftQuarter,
    BooleanSupplier centricToggle) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;
        this.POV = POV;
        this.shiftHalf = shiftHalf;
        this.shiftQuarter = shiftQuarter;
        this.centricToggle = centricToggle;
        
        rotationSpeed = 0;
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        usePOV = false;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;
        
        switch ((int) (POV.getAsDouble())) {
            case Constants.FORWARD:
            headingY = 1;
            break;
            case Constants.FORWARD_RIGHT:
            headingX = -1;
            headingY = 1;
            break;
            case Constants.RIGHT:
            headingX = -1;
            break;
            case Constants.BACKWARD_RIGHT:
            headingX = -1;
            headingY = -1;
            break;
            case Constants.BACKWARD:
            headingY = -1;
            break;
            case Constants.BACKWARD_LEFT:
            headingX = 1;
            headingY = -1;
            break;
            case Constants.LEFT:
            headingX = 1;
            break;
            case Constants.FORWARD_LEFT:
            headingX = 1;
            headingY = 1;
            break;
        }
        
        if (POV.getAsDouble() != -1) {
            usePOV = true;
        }
        
        if (Math.abs(heading.getAsDouble()) > swerve.getSwerveController().config.angleJoyStickRadiusDeadband) {
            rotationSpeed = heading.getAsDouble() * swerve.getSwerveController().config.maxAngularVelocity;
            usePOV = false;
        } else {
            rotationSpeed = 0;
        }
        
        boolean hasAlliance = DriverStation.getAlliance().isPresent();
        if(hasAlliance && isFieldCentric && DriverStation.getAlliance().get() == Alliance.Red) {
            allianceCorrection = -1;
        }
        else { 
            allianceCorrection = 1;
        }
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble() * allianceCorrection, vY.getAsDouble() * allianceCorrection, headingX, headingY);
        //ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), new Rotation2d(swerve.getHeading().getRadians()));
        if (centricToggle.getAsBoolean()) {
            isFieldCentric = !isFieldCentric;
        }
        // Limit velocity to prevent tippy
        Translation2d translation = new Translation2d(vX.getAsDouble()*Constants.DrivebaseConstants.MAX_SPEED_FEET_PER_SECOND * allianceCorrection, vY.getAsDouble()*Constants.DrivebaseConstants.MAX_SPEED_FEET_PER_SECOND * allianceCorrection);
        //translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        //   Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        //  swerve.getSwerveDriveConfiguration());
        //SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        //SmartDashboard.putString("Translation", translation.toString());
        
        double multiplier = shiftQuarter.getAsBoolean() ? 0.25 : (shiftHalf.getAsBoolean() ? 0.5 : 1);
        translation = translation.times(multiplier);
        rotationSpeed = rotationSpeed * multiplier;
        
        // Make the robot move
        if (usePOV) {
            swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, isFieldCentric);
        } else {
            swerve.drive(translation, rotationSpeed, isFieldCentric);
        }
        // swerve.drive(translation, rotationSpeed, isFieldCentric);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
