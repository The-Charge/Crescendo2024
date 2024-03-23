// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FixedLLConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToTagCommandGroup extends SequentialCommandGroup{
    public DriveToTagCommandGroup(VisionSubsystem limelight, SwerveSubsystem swerve){

            addCommands(
                new SetCurrentCamera(limelight, FixedLLConstants.FIXED_LL_NAME),
                new SetPipeline(limelight, FixedLLConstants.FIXED_APRILTAG_PIPELINE),
                new UpdateRobotPose(swerve, limelight),
                new DriveToTag(swerve, limelight, limelight.gettid(), false),
                new SetPriorityID(limelight, limelight.getprevtag()),
                new TagCorrection(swerve, limelight),
                new SetPriorityID(limelight, -1)
            );  
            
           
        
    }
  }
