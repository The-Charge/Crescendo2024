// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * An example command that uses an example subsystem.
 */

public class DriveToNote extends InstantCommand {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem limelight;
    private final PIDController heading_controller;
    private final PIDController drive_controller;
    private Timer detectiontimer;
    private boolean timerstarted = false;
    public DriveToNote(SwerveSubsystem swerve, VisionSubsystem limelight){
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
        addRequirements(limelight);
        heading_controller = new PIDController(0.009, 0.0, 0.0);
        heading_controller.setTolerance(VisionConstants.TX_TOLERANCE_THRESHOLD);
        heading_controller.setSetpoint(0.0);

        drive_controller = new PIDController(0.5, 0.0, 0.0);
        drive_controller.setTolerance(VisionConstants.TARGET_DISTANCE_TOLERANCE_THRESHOLD);
        drive_controller.setSetpoint(0.0);

    }

 


@Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double tx = limelight.gettx();
    double distance = limelight.getdistance();
    heading_controller.reset();
    drive_controller.reset();
    double RotationVal = MathUtil.clamp(heading_controller.calculate(tx, 0.0), -1, 1);
    double TranslationVal = MathUtil.clamp(drive_controller.calculate(distance, 0.0), -0.5, 0.5);

    if (limelight.gettv() > 0.0){
      timerstarted = false;
      if (detectiontimer != null){
        detectiontimer.stop();
      }
      
      swerve.drive(new Translation2d(-1 * TranslationVal * 14.5,0), RotationVal * swerve.getSwerveController().config.maxAngularVelocity, false);
    }
    if (limelight.gettv() < 0.0){
       detectiontimer = new Timer();
       detectiontimer.start();
       timerstarted = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    heading_controller.reset();
    drive_controller.reset();
    swerve.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (detectiontimer == null){
      return false;
    }  
    else{
      return detectiontimer.hasElapsed(0.2);
    }
  }

}