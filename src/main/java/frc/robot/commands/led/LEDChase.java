package frc.robot.commands.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStripSubsystem;

public class LEDChase extends Command {
    
    private final LEDStripSubsystem strip;
    private final BooleanSupplier end;
    private final Color col1, col2;

    /**
     * Makes a moving chase pattern (segments of each color moving down the strip)
     * @param nStrip LED Strip Subsystem
     * @param nCol1 the first color in the sequence
     * @param nCol2 the second color in the sequence
     * @param endCondition a BooleanSupplier that when true will end the command
     */
    public LEDChase(LEDStripSubsystem nStrip, Color nCol1, Color nCol2, BooleanSupplier endCondition) {
        strip = nStrip;
        addRequirements(strip);

        col1 = nCol1;
        col2 = nCol2;
        end = endCondition;
    }

    @Override
    public void initialize() {
        strip.resetAnimationTimer();
    }
    @Override
    public void execute() {
        strip.chase(col1, col2);
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
