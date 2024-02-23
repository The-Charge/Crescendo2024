// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTarget extends InstantCommand {
    private final SwerveSubsystem swerve;
    public DriveToTarget(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

 


@Override
  public void initialize() {

}

  @Override
  public void execute() {
    double ta = RobotContainer.getLimelight().getta();
    swerve.drive(new Translation2d(0.1 * 14.5, 0), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.getLimelight().gettv() < 1.0;
  }

}
