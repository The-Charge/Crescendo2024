// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTag extends InstantCommand {
    private final SwerveSubsystem swerve;
    private Command drivetoPose;
  
    public DriveToTag(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

 


@Override
  public void initialize() {
    if (RobotContainer.getLimelight().gettv() > 0.0){
      drivetoPose = swerve.driveToPose(Constants.ApriltagConstants.OFFSET_APRILTAG_POSE[(int)RobotContainer.getLimelight().gettid()]);
      drivetoPose.schedule();
    }
    
}

  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetoPose != null && drivetoPose.isFinished();
  }

}