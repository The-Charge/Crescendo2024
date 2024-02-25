// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* */
package frc.robot.commands.led;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDStripSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class LEDVision extends Command {

  private final LEDStripSubsystem strip;
  private final DoubleSupplier tv;
    private int stripSize;

  public LEDVision(LEDStripSubsystem sub1, DoubleSupplier tv) {
    strip = sub1;
    this.tv = tv;
    addRequirements(sub1);
  }

 


@Override
  public void initialize() {
   
  }

  @Override
  public void execute() {
    strip.setVisionPixelRGB();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
