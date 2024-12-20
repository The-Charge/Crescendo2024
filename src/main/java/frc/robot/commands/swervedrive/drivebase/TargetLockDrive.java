// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;

import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class TargetLockDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY, heading;
  private double rotationSpeed;
  private final PIDController heading_controller;
  private final boolean fieldRelative;
  private final VisionSubsystem limelight;
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
   * @param heading DoubleSupplier that supplies the robot's heading angle. Will be corrected with rotation in this.
   */
  public TargetLockDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading, boolean fieldRelative, VisionSubsystem limelight) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.fieldRelative = fieldRelative;
    this.limelight = limelight;
    rotationSpeed = 0;
    heading_controller = new PIDController(0.015, 0.0005, 0.0003);
    heading_controller.setTolerance(VisionConstants.TX_TOLERANCE_THRESHOLD);
    heading_controller.setSetpoint(0.0);
    addRequirements(swerve);
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = limelight.gettx();
    double RotationVal = MathUtil.clamp(heading_controller.calculate(tx, 0.0), -1, 1);
    if (limelight.gettv() > 0.0)
    rotationSpeed = RotationVal * swerve.getSwerveController().config.maxAngularVelocity;
    else if (Math.abs(heading.getAsDouble()) > swerve.getSwerveController().config.angleJoyStickRadiusDeadband) {
      rotationSpeed = heading.getAsDouble()*swerve.getSwerveController().config.maxAngularVelocity;
    }
    else {
      rotationSpeed = 0;
    }
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble() * 0.75, vY.getAsDouble() * 0.75, new Rotation2d(rotationSpeed));
    
    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, rotationSpeed, fieldRelative);
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
