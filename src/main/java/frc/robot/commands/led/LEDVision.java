package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStripSubsystem;

public class LEDVision extends Command {
    
    private final LEDStripSubsystem strip;

    /**
     * Disables all of the LEDs (sets them to #000000)
     */
    public LEDVision(LEDStripSubsystem nStrip) {
        strip = nStrip;

        addRequirements(strip);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
         strip.setVisionPixelRGB();
    }
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
