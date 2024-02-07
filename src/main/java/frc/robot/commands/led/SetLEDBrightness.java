package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDStripSubsystem;

public class SetLEDBrightness extends Command {
    
    private final LEDStripSubsystem strip;
    private final double bright;

    /**
     * Sets the brightness modifier of the LED strip
     * @param nStrip LED strip subsystem
     * @param brightness Range of 0 - 1
     */
    public SetLEDBrightness(LEDStripSubsystem nStrip, double brightness) {
        strip = nStrip;
        bright = brightness;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        strip.setBrightness(bright);
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
