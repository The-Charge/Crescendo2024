// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStripSubsystem extends SubsystemBase {

  private AddressableLED leds;
  private AddressableLEDBuffer buffer;
  private boolean hasChanged = true; //we want to update the LEDs on the first frame
  private double brightness = 1;

  /**
   * Defaults to using a 45 pixel-long strip. Uses port 9 for PWM control
   */
  public LEDStripSubsystem() {
    leds = new AddressableLED(9);

    buffer = new AddressableLEDBuffer(45);
    leds.setLength(buffer.getLength());

    leds.setData(buffer);
    leds.start();
  }
  /**
   * Uses port 9 for PWM control
   * @param pixelCount Number of pixels/LEDs in the strip. For chained strips, add the number of pixels in each strip
   */
  public LEDStripSubsystem(int pixelCount) {
    leds = new AddressableLED(9);

    buffer = new AddressableLEDBuffer(pixelCount);
    leds.setLength(buffer.getLength());

    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    if(hasChanged) {
      //only update LEDs when the buffer is changed, so we don't waste processing power on it
      leds.setData(buffer);
      hasChanged = false;
    }
  }
  @Override
  public void simulationPeriodic() {}

  public void setPixelColor(int pixel, Color col) {
    setPixelColor(pixel, col, false);
  }
  public void setPixelColor(int pixel, Color col, boolean ignoreBrightness) {
    setPixelRGB(
      pixel,
      (int) (col.red * 255),
      (int) (col.green * 255),
      (int) (col.blue * 255),
      ignoreBrightness
    );

    hasChanged = true;
  }

  public void setPixelRGB(int pixel, int r, int g, int b) {
    setPixelRGB(pixel, r, g, b, false);
  }
  public void setPixelRGB(int pixel, int r, int g, int b, boolean ignoreBrightness) {
    buffer.setRGB(
      pixel,
      (int) (g * (ignoreBrightness ? 1 : brightness)),
      (int) (r * (ignoreBrightness ? 1 : brightness)),
      (int) (b * (ignoreBrightness ? 1 : brightness))
    ); //led strips use GRB mapping

    hasChanged = true;
  }
  /**
   * Sets the brightness modifier
   * @param nBrightness In range of 0 - 1
   */
  public void setBrightness(double nBrightness) {
    brightness = Math.max(Math.min(nBrightness, 1), 0);
  }
  public void requestUpdate() {
    hasChanged = true;
  }

  public int getStripLength() {
    return buffer.getLength();
  }
  public Color getPixelColor(int pixel) {
    return buffer.getLED(pixel);
  }
  public double getBrightness() {
    return brightness;
  }
}
