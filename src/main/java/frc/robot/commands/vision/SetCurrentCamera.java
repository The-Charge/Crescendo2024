// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class SetCurrentCamera extends InstantCommand {
    private final VisionSubsystem limelight;
    private final String limelightname;
    public SetCurrentCamera(VisionSubsystem limelight, String limelightname){
        this.limelight = limelight;
        this.limelightname = limelightname;
        addRequirements(limelight);
    }

 


@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    limelight.setCurrentLimelightName(limelightname);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
