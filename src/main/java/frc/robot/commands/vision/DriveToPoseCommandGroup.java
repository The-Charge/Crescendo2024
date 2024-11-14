// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
/**
 * An example command that uses an example subsystem.
 */

public class DriveToPoseCommandGroup extends SequentialCommandGroup{
    public DriveToPoseCommandGroup(VisionSubsystem limelight, SwerveSubsystem swerve, Pose2d pose){
            addCommands(
                new UpdateRobotPose(swerve, limelight),
                new DriveToPose(swerve, limelight, pose)
                //Field relative moves "freely", robot relative creates a radius around speaker depending on where command is initiated
            );  
            
           
        
    }
  }
