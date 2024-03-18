// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToPose extends InstantCommand {
    private final SwerveSubsystem swerve;
    private Command drivetoPose;
    private double tag;
    private boolean ManualControl;
    private VisionSubsystem limelight;
    private final Pose2d pose;
    public DriveToPose(SwerveSubsystem swerve, VisionSubsystem limelight, Pose2d pose){
        this.swerve = swerve;
        this.limelight = limelight;
        this.pose = pose;
        addRequirements(swerve);
        addRequirements(limelight);
    }
 


@Override
  public void initialize() {
        drivetoPose = swerve.driveToPose(pose);
        drivetoPose.schedule();
        
      
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
