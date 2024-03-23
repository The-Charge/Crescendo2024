// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDStripSubsystem extends SubsystemBase {
    
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;
    private boolean hasChanged = true; //we want to update the LEDs on the first frame
    private double brightness = 1;
    private int animationTimer = 0;
    
    public LEDStripSubsystem() {
        leds = new AddressableLED(Constants.LEDConstants.ledId);
        
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
        //led strips use GRB mapping
        buffer.setRGB(pixel, (int) (g * (ignoreBrightness ? 1 : brightness)), (int) (r * (ignoreBrightness ? 1 : brightness)), (int) (b * (ignoreBrightness ? 1 : brightness)));
        
        hasChanged = true;
    }
    
    public void setRange(int start, int end, Color col) {
        for(int i = start; i < end; i++) {
            setPixelColor(i, col);
        }
    }
    public void fill(Color col) {
        setRange(0, getStripLength(), col);
    }
    public void resetAnimationTimer() {
        animationTimer = 0;
    }
    public void rainbow() {
        final int s = 255, v = 255;
        final double speedModifier = 3;
        final int pixelDistance = 3;
        
        for(int i = 0; i < getStripLength(); i++) {
            setPixelHSV(i, (int) (animationTimer * speedModifier + i * pixelDistance) % 180, s, v);
        }
        
        animationTimer++;
    }
    public void chase(Color col1, Color col2) {
        final int segmentLength = 15;
        final double speedModifier = 0.3;
        
        for(int i = 0; i < getStripLength(); i++) {
            setPixelColor(i, (Math.floor((i + animationTimer * speedModifier) / segmentLength) % 2 == 0 ? col1 : col2));
        }
        
        animationTimer++;
    }
    public void blink(Color col) {
        final int onLength = 15;
        final int offLength = 15;
        
        for(int i = 0; i < getStripLength(); i++) {
            setPixelColor(i, (animationTimer % (onLength + offLength) < onLength ? col : Color.kBlack));
        }
        
        animationTimer++;
    }
    
    /**
    * Sets the brightness modifier
    * @param nBrightness In range of 0 - 1
    * @param shouldUpdate whether to update the strip with the new brightness immediately
    */
    public void setBrightness(double nBrightness, boolean shouldUpdate) {
        brightness = Math.max(Math.min(nBrightness, 1), 0);
        
        if(shouldUpdate) {
            for(int i = 0; i < getStripLength(); i++) {
                Color col = getPixelColor(i);
                setPixelColor(i, col, false); //reset the color, but factor in brightness
            }
            
            hasChanged = true;
        }
    }

    public void setVisionPixelRGB(){
        /*
        if (RobotContainer.getlimelight().gettv() > 0 && (RobotContainer.getCollectorHeadSubsystem().getNoteSensor1() || RobotContainer.getCollectorHeadSubsystem().getNoteSensor2())){
          setRange(0, getStripLength(), Color.kGold);
        }
        else if (RobotContainer.getlimelight().gettv() > 0 && (!RobotContainer.getCollectorHeadSubsystem().getNoteSensor1() || !RobotContainer.getCollectorHeadSubsystem().getNoteSensor2())){
          setRange(0, getStripLength(), Color.kOrange);
        }
       else{
          setRange(0, getStripLength(), Color.kGreen);
        }
         */
        if (RobotContainer.getCollectorHeadSubsystem().getNoteSensor1() || RobotContainer.getCollectorHeadSubsystem().getNoteSensor2()){
          setRange(0, getStripLength(), Color.kLime);
        }
         else if (RobotContainer.getlimelight().gettv() > 0){
          /*for(int i = 0; i < getStripLength(); i++) {
            setPixelRGB(i, 255, 0, 0, false);
          }*/
        //   setRange(0, getStripLength(), Color.kGold);
          blink(Color.kGold);
        }
        else{
          chase(Color.kLime, Color.kGold);
       }
   
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
