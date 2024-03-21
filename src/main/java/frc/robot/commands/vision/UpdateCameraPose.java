// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class UpdateCameraPose extends Command {
    private final VisionSubsystem limelight;
  
  
    public UpdateCameraPose(VisionSubsystem limelight){
        this.limelight = limelight;
        addRequirements(limelight);
    }

 


@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    limelight.setCameraPose();
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
