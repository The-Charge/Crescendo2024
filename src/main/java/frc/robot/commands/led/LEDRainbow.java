package frc.robot.commands.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStripSubsystem;

public class LEDRainbow extends Command {
    
    private final LEDStripSubsystem strip;
    private final BooleanSupplier end;

    /**
     * Makes a moving rainbow pattern across the whole strip
     * @param nStrip LED Strip Subsystem
     * @param endCondition a BooleanSupplier that when true will end the command
     */
    public LEDRainbow(LEDStripSubsystem nStrip, BooleanSupplier endCondition) {
        strip = nStrip;
        addRequirements(strip);

        end = endCondition;
    }

    @Override
    public void initialize() {
        strip.resetRainbowTimer();
    }
    @Override
    public void execute() {
        strip.rainbow();
    }
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return end.getAsBoolean();
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
