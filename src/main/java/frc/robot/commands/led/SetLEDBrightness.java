package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStripSubsystem;

public class SetLEDBrightness extends Command {
    
    private final LEDStripSubsystem strip;
    private final double bright;
    private final boolean update;

    /**
     * Sets the brightness modifier of the LED strip
     * @param nStrip LED strip subsystem
     * @param brightness Range of 0 - 1
     * @param updateStrip If true, will update the color of every pixel to match the new brightness
     */
    public SetLEDBrightness(LEDStripSubsystem nStrip, double brightness, boolean updateStrip) {
        strip = nStrip;
        bright = brightness;
        update = updateStrip;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        strip.setBrightness(bright, update);
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
