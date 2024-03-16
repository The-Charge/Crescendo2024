// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FixedLLConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class AutonDriveToTag extends SequentialCommandGroup{
    public AutonDriveToTag(VisionSubsystem limelight, SwerveSubsystem swerve, double settag){

            addCommands(
                new SetCurrentCamera(limelight, FixedLLConstants.FIXED_LL_NAME),
                new SetPipeline(limelight, VisionConstants.APRILTAG_PIPELINE),
                new UpdateRobotPose(swerve, limelight),
                new DriveToTag(swerve, limelight, settag, true),
                new SetPriorityID(limelight, limelight.getprevtag()),
                new TagCorrection(swerve, limelight),
                new SetPriorityID(limelight, -1)
            );  
            
           
        
    }
  }
