// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTag extends InstantCommand {
    private final VisionSubsystem limelight;
    private final SwerveSubsystem swerve;
  
    public DriveToTag(VisionSubsystem limelight, SwerveSubsystem swerve){
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(limelight);
    }

 


@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (limelight.gettv() > 0){
        Commands.deferredProxy(() -> swerve.driveToPose(Constants.ApriltagConstants.APRILTAG_POSE[(int)limelight.gettid()]));
    }
    else{
        System.out.println("No tag found");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
