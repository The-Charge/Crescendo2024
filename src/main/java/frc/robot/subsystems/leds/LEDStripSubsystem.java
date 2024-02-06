// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.w3c.dom.ls.LSException;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putNumber("red color", getPixelColor(0).red);
    SmartDashboard.putNumber("blue color", getPixelColor(0).blue);
    SmartDashboard.putNumber("green color", getPixelColor(0).green);
  }
  @Override
  public void simulationPeriodic() {}

  public void setBufferPixel(int pixel, Color col) {
    // buffer.setLED(pixel, col);
    setBufferRGB(pixel, (int) (col.red * 255), (int) (col.green * 255), (int) (col.blue * 255));
  }
  public void setBufferHSV(int pixel, int h, int s, int v) {
    buffer.setHSV(pixel, (int) (h - (180 / 3.0)), s, v);
  }
  public void setBufferRGB(int pixel, int r, int g, int b) {
    buffer.setRGB(pixel, g, r, b);  //LED is in GRB?
  }

  public int getStripLength() {
    return buffer.getLength();
  }  
  public Color getPixelColor(int pixel) {
    return buffer.getLED(pixel);
  }
}
