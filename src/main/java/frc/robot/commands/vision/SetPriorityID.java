// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class SetPriorityID extends Command {
    private final VisionSubsystem limelight;
    private final double id;    //most likely will be tag 8, set to -1 if none
  
    public SetPriorityID(VisionSubsystem limelight, double id){
        this.id = id;
        this.limelight = limelight;
        addRequirements(limelight);
    }

 


@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (RobotContainer.getlimelight().getlimelightshootername() == "limelight-fixed"){
      limelight.setPriorityID(id);
    }
    
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
