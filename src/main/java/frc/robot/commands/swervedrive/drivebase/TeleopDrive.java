// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.*;
import frc.robot.commands.StateMachine;
import frc.robot.commands.Elevator.MoveElevatorToSetpoint;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.*;

import javax.naming.spi.StateFactory;

import frc.robot.Constants;
import frc.robot.Constants.StateLocations;

import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class TeleopDrive extends Command {

  private final SwerveSubsystem swerve;
  private final ElevatorSubsystem elev;
  private final PivotSubsystem pivot;
  private final DoubleSupplier vX, vY, heading;
  private double rotationSpeed;
  private final BooleanSupplier startup, pickupFloor, pickupSource, shootAmpTrap, shootHighRear, shootShallowFront, shootSteepFront, travel;

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
  public TeleopDrive(SwerveSubsystem swerve, ElevatorSubsystem elevSub, PivotSubsystem pivSub, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, BooleanSupplier goToStartup, BooleanSupplier goToPickupFloor, BooleanSupplier goToPickupSource, BooleanSupplier goToShootAmpTrap, BooleanSupplier goToShootHighRear, BooleanSupplier goToShootShallowFront, BooleanSupplier goToShootSteepFront, BooleanSupplier goToTravel) {
    this.swerve = swerve;
    this.elev = elevSub;
    this.pivot = pivSub;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    rotationSpeed = 0;

    addRequirements(swerve);

    this.startup = goToStartup;
    this.pickupFloor = goToPickupFloor;
    this.pickupSource = goToPickupSource;
    this.shootAmpTrap = goToShootAmpTrap;
    this.shootHighRear = goToShootHighRear;
    this.shootShallowFront = goToShootShallowFront;
    this.shootSteepFront = goToShootSteepFront;
    this.travel = goToTravel;
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(heading.getAsDouble()) > swerve.getSwerveController().config.angleJoyStickRadiusDeadband) {
      rotationSpeed = heading.getAsDouble()*swerve.getSwerveController().config.maxAngularVelocity;
    }
    else {
      rotationSpeed = 0;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), new Rotation2d(rotationSpeed));
    
    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, rotationSpeed, true);

    if(startup.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.STARTUP);
    else if(pickupFloor.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.PICKUPFLOOR);
    else if(pickupSource.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.PICKUPSOURCE);
    else if(shootAmpTrap.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.SHOOTAMPTRAP);
    else if(shootHighRear.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.SHOOTHIGHREAR);
    else if(shootShallowFront.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.SHOOOTSHALLOWFRONT);
    else if(shootSteepFront.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.SHOOTSTEEPFRONT);
    else if(travel.getAsBoolean()) new StateMachine(elev, pivot, StateMachine.State.TRAVEL);
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
