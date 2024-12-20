// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterLLConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToNoteCommandGroup extends SequentialCommandGroup{
    public DriveToNoteCommandGroup(VisionSubsystem limelight, SwerveSubsystem swerve){

            addCommands(
                new SetCurrentCamera(limelight, ShooterLLConstants.SHOOTER_LL_NAME),
                new SetPipeline(limelight, ShooterLLConstants.SHOOTER_NEURAL_NETWORK_PIPELINE),
                new DriveToNote(swerve, limelight)
            );  
            
           
        
    }
  }
