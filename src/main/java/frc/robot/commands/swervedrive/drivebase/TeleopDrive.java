// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
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
      DoubleSupplier heading, DoubleSupplier POV, BooleanSupplier shiftHalf, BooleanSupplier shiftQuarter, BooleanSupplier centricToggle) {
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

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

    double multiplier = shiftQuarter.getAsBoolean() ? 0.25 : (shiftHalf.getAsBoolean() ? 0.5 : 1);
    desiredSpeeds = desiredSpeeds.times(multiplier);

    if(centricToggle.getAsBoolean()) {
      isFieldCentric = !isFieldCentric;
    }
    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    if (usePOV) {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, isFieldCentric);
    } else {
      swerve.drive(translation, rotationSpeed, isFieldCentric);
    }
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
