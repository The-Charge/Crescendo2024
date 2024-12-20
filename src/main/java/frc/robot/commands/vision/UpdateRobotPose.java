// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class UpdateRobotPose extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem limelight;
    public UpdateRobotPose(SwerveSubsystem swerve, VisionSubsystem limelight){
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
    }

 


@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    swerve.addVisionReading(limelight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Ensures robot pose estimator is within pose given by limelight
    return swerve.updatedPoseWithinThreshold(limelight);
  }

}
