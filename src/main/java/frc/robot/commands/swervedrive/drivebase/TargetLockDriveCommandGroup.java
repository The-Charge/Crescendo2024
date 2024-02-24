// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.vision.DriveToTarget;
import frc.robot.commands.vision.SetPipeline;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class TargetLockDriveCommandGroup extends SequentialCommandGroup{
    public TargetLockDriveCommandGroup(VisionSubsystem limelight, SwerveSubsystem swerve,  DoubleSupplier vX, DoubleSupplier vY){

            addCommands(
                new SetPipeline(limelight, VisionConstants.SPEAKER_LOCK_PIPELINE),
                new TargetLockDrive(swerve, vX, vY)
            );  
            
           
        
    }
  }
