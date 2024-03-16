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

public class TagCorrection extends InstantCommand {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem limelight;
    private final PIDController heading_controller;
    private double tx;
    private Timer detectiontimer;
    private boolean timerstarted = false;
    public TagCorrection(SwerveSubsystem swerve, VisionSubsystem limelight){
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve);
        addRequirements(limelight);
        heading_controller = new PIDController(0.009, 0.0, 0.0);
        heading_controller.setTolerance(VisionConstants.TX_TOLERANCE_THRESHOLD);
        heading_controller.setSetpoint(0.0);

    }

 


  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    tx = limelight.gettx();
    heading_controller.reset();
    double TranslationVal = MathUtil.clamp(heading_controller.calculate(tx, 0.0), -1, 1);
    swerve.drive(new Translation2d(0, -1 * TranslationVal * 14.0), 0, false);  
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    heading_controller.reset();
    swerve.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return tx < VisionConstants.TX_TOLERANCE_THRESHOLD && tx > - 1 * VisionConstants.TX_TOLERANCE_THRESHOLD;
  }

}
