// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDStripSubsystem extends SubsystemBase {

  private AddressableLED leds;
  private AddressableLEDBuffer buffer;
  private boolean hasChanged = true; //we want to update the LEDs on the first frame
  private double brightness = 1;
  private int rainbowTimer = 0;

  public LEDStripSubsystem() {
    leds = new AddressableLED(Constants.LEDConstants.portId);

    buffer = new AddressableLEDBuffer(Constants.LEDConstants.totalLength);
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
    setPixelRGB(pixel, (int) (col.red * 255), (int) (col.green * 255), (int) (col.blue * 255), ignoreBrightness);
  }

  /**
   * @param h 0 - 180
   * @param s 0 - 255
   * @param v 0 - 255
  */
  public void setPixelHSV(int pixel, int h, int s, int v) {
    setPixelHSV(pixel, h, s, v, false);
  }
  /**
   * @param h 0 - 180
   * @param s 0 - 255
   * @param v 0 - 255
  */
  public void setPixelHSV(int pixel, int h, int s, int v, boolean ignoreBrightness) {
    setPixelColor(pixel, Color.fromHSV(h, s, v), ignoreBrightness);
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

  public void setRange(int start, int end, Color col) {
    for(int i = start; i < end; i++) {
      setPixelColor(i, col);
    }
  }
  public void fill(Color col) {
    setRange(0, buffer.getLength(), col);
  }
  public void resetRainbowTimer() {
    rainbowTimer = 0;
  }
  public void rainbow() {
    final int s = 255, v = 255;
    final int speedModifier = 3;
    final int pixelDistance = 3;

    for(int i = 0; i < buffer.getLength(); i++) {
      setPixelHSV(i, (rainbowTimer * speedModifier + i * pixelDistance) % 180, s, v);
    }

    rainbowTimer++;
  }

  /**
   * Sets the brightness modifier
   * @param nBrightness In range of 0 - 1
  */
  public void setBrightness(double nBrightness) {
    brightness = Math.max(Math.min(nBrightness, 1), 0);
  }
  /**
   * Manually request and update to the LEDs
  */
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