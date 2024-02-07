package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDStripSubsystem;

public class SetLEDRange extends Command {
    
    private final LEDStripSubsystem strip;
    private final int start, end;
    private final Color col;

    /**
     * Sets a certain segment of a strip to a solid color
     * @param nStrip LED strip subsystem
     * @param rangeStart Index of the beginning of the segment (inclusive)
     * @param rangeEnd Index of the end of the segment (exclusive)
     * @param nCol The color to set the LEDs to
     */
    public SetLEDRange(LEDStripSubsystem nStrip, int rangeStart, int rangeEnd, Color nCol) {
        strip = nStrip;
        start = rangeStart;
        end = rangeEnd;
        col = nCol;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        for(int i = start; i < end; i++) {
            strip.setPixelColor(i, col);
        }
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
