package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStripSubsystem;

public class DisableLEDs extends Command {
    
    private final LEDStripSubsystem strip;

    /**
     * Disables all of the LEDs (sets them to #000000)
     */
    public DisableLEDs(LEDStripSubsystem nStrip) {
        strip = nStrip;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        strip.fill(new Color(0, 0, 0));
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
