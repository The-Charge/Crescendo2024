// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.leds.LEDStripSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RunLEDExample extends Command {

  private final LEDStripSubsystem strip;
  private final DoubleSupplier joyX, joyY;
  private final Trigger but1;

  public RunLEDExample(LEDStripSubsystem sub1, DoubleSupplier nJoyX, DoubleSupplier nJoyY, Trigger nBut1) {
    strip = sub1;
    joyX = nJoyX;
    joyY = nJoyY;
    but1 = nBut1;

    addRequirements(sub1);
  }

  @Override
  public void initialize() {
    int stripSize = strip.getStripLength();
    int hue = (int) ((-joyY.getAsDouble() / 2.0 + 0.5) * 180);
    int value = (int) ((joyX.getAsDouble() / 2.0 + 0.5) * 255);

    int rr  = (int) (Math.random() * 255);
    int rg = (int) (Math.random() * 255);
    int rb = (int) (Math.random() * 255);
    for(int i = 0; i < stripSize; i++) {
      if(but1.getAsBoolean()) {
        strip.setBufferRGB(i, rr, rg, rb);
      }
      else {
        // strip.setBufferHSV(i, hue, 255, value);
      }
    }
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
