// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.subsystems.VisionSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class SetCenterSpeakerPriorityID extends Command {
    private final VisionSubsystem limelight;
    public SetCenterSpeakerPriorityID(VisionSubsystem limelight){
        this.limelight = limelight;
        addRequirements(limelight);
    }

@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Red){
      limelight.setPriorityID(ApriltagConstants.RED_SPEAKER_CENTER_TAG);
    }
    if (ally.get() == Alliance.Blue){
      limelight.setPriorityID(ApriltagConstants.BLUE_SPEAKER_CENTER_TAG);
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
