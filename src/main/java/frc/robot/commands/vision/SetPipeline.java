// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class SetPipeline extends InstantCommand {
    private final VisionSubsystem limelight;
    private final double index;
    public SetPipeline(VisionSubsystem limelight, double index){
        this.limelight = limelight;
        this.index = index;
        addRequirements(limelight);
    }

 


@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    limelight.setPipeline(index);
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
