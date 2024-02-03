// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.w3c.dom.ls.LSException;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStripSubsystem extends SubsystemBase {

  private AddressableLED leds;
  private AddressableLEDBuffer buffer;

  /**
   * Defaults to using a 45 pixel-long strip
   */
  public LEDStripSubsystem() {
    leds = new AddressableLED(9);

    buffer = new AddressableLEDBuffer(45);
    leds.setLength(buffer.getLength());

    leds.setData(buffer);
    leds.start();
  }
  public LEDStripSubsystem(int pixelCount) {
    leds = new AddressableLED(9);

    buffer = new AddressableLEDBuffer(pixelCount);
    leds.setLength(buffer.getLength());

    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    leds.setData(buffer);
  }
  @Override
  public void simulationPeriodic() {}

  public void setBufferPixel(int pixel, Color col) {
    buffer.setLED(pixel, col);
  }
  public void setBufferHSV(int pixel, int h, int s, int v) {
    buffer.setHSV(pixel, h, s, v);
  }
  public void setBufferRGB(int pixel, int r, int g, int b) {
    buffer.setRGB(pixel, r, g, b);
  }

  public int getStripLength() {
    return buffer.getLength();
  }
  public Color getPixelColor(int pixel) {
    return buffer.getLED(pixel);
  }
}
