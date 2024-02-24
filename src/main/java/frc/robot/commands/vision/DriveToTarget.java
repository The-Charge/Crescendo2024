// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTarget extends InstantCommand {
    private final SwerveSubsystem swerve;
    private final PIDController heading_controller;
    private final PIDController drive_controller;
    public DriveToTarget(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
        heading_controller = new PIDController(0.01, 0.0, 0.0);
        heading_controller.setTolerance(VisionConstants.TX_THRESHOLD);
        heading_controller.setSetpoint(0.0);

        drive_controller = new PIDController(1, 0.0, 0.0);
        drive_controller.setTolerance(0.5);
        drive_controller.setSetpoint(0.0);

    }

 


@Override
  public void initialize() {

}

  @Override
  public void execute() {
    double tx = RobotContainer.getLimelight().gettx();
    double distance = RobotContainer.getLimelight().getdistance();
    double RotationVal = MathUtil.clamp(heading_controller.calculate(tx, 0.0), -0.4, 0.4);
    double TranslationVal = MathUtil.clamp(drive_controller.calculate(distance, 0.0), -0.1, 0.1);
    SmartDashboard.putNumber("TranslationVal VISION:", TranslationVal);
    swerve.drive(new Translation2d(-1 * TranslationVal * 14.5,0), RotationVal * swerve.getSwerveController().config.maxAngularVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.getLimelight().gettv() < 1.0;
  }

}
