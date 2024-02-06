// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.leds.LEDStripSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class LEDAprilTag extends Command {

  private final LEDStripSubsystem strip;
  private final DoubleSupplier tv, tx;
    private int stripSize;

  public LEDAprilTag(LEDStripSubsystem sub1, DoubleSupplier tv, DoubleSupplier tx) {
    strip = sub1;
    this.tv = tv;
    this.tx = tx;
    addRequirements(sub1);
  }

 


@Override
  public void initialize() {
    stripSize = strip.getStripLength();

    int rr  = (int) (255);
    int rg = (int) (0);
    int rb = (int) (0);
    for(int i = 0; i < stripSize; i++) {
        strip.setBufferRGB(i, rr, rg, rb);
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("tv LED", tv.getAsDouble());
    SmartDashboard.putNumber("tx LED", tx.getAsDouble());
    if (tv.getAsDouble() > 0){
          for(int i = 0; i < stripSize; i++) {
        strip.setBufferRGB(i, (int) 255,(int) 0, (int)0);
        }
    }
    else{
         for(int i = 0; i < stripSize; i++) {
        strip.setBufferRGB(i, (int)255, (int)0,(int) 255);
    }
    }
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
