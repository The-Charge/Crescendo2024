// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDStripSubsystem;

public class RunLEDExample extends Command {

  private final LEDStripSubsystem strip;

  /**
   * Rapidly flashes the LED strip. DO NOT USE AROUND PHOTOSENSITIVE PEOPLE
   * @param stripSub LED strip subsystem
   */
  public RunLEDExample(LEDStripSubsystem stripSub) {
    strip = stripSub;

    addRequirements(stripSub);
  }

  @Override
  public void initialize() {
    int stripSize = strip.getStripLength();

    int rr  = (int) (Math.random() * 255);
    int rg = (int) (Math.random() * 255);
    int rb = (int) (Math.random() * 255);
    for(int i = 0; i < stripSize; i++) {
      strip.setPixelRGB(i, rr, rg, rb);
    }
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return true;}
}
