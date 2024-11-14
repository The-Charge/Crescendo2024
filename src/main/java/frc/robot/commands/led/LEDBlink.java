package frc.robot.commands.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStripSubsystem;

public class LEDBlink  extends Command {
    
    private LEDStripSubsystem led;
    private Color col;
    private BooleanSupplier end;

    public LEDBlink(LEDStripSubsystem strip, Color col, BooleanSupplier endCondition) {
        this.led = strip;
        addRequirements(this.led);

        this.col = col;
        this.end = endCondition;
    }

    @Override
    public void initialize() {
        led.resetAnimationTimer();
    }
    @Override
    public void execute() {
        led.blink(col);
    }
    @Override
    public boolean isFinished() {
        return end.getAsBoolean();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
