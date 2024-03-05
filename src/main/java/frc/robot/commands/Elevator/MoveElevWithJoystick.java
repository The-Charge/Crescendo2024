package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevWithJoystick extends Command {

    public final ElevatorSubsystem elev;
    public final DoubleSupplier joyY;

    public MoveElevWithJoystick(ElevatorSubsystem elev, DoubleSupplier joystickY) {
        this.elev = elev;
        addRequirements(elev);

        this.joyY = joystickY;
    }

    @Override
    public void execute() {
        elev.goToPosition((-joyY.getAsDouble() + 1) / 2.0 * 27);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
