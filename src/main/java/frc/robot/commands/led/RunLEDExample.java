// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDStripSubsystem;

public class RunLEDExample extends Command {

  private final LEDStripSubsystem strip;

  private Color rgb;

  /**
   * Rapidly flashes the LED strip. DO NOT USE AROUND PHOTOSENSITIVE PEOPLE
   * 
   * @param stripSub LED strip subsystem
   */
  public RunLEDExample(LEDStripSubsystem stripSub, int hue, int saturation, int value) {
    strip = stripSub;
    rgb = Color.fromHSV(hue, saturation, value);
    addRequirements(stripSub);
  }

  @Override
  public void initialize() {
    int stripSize = strip.getStripLength();

    int red = (int) (rgb.red);
    int green = (int) (rgb.green);
    int blue = (int) (rgb.blue);

    for (int i = 0; i < stripSize; i++) {
      strip.setPixelRGB(i, red, green, blue);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
