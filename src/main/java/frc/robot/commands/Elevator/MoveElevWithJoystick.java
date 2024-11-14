package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevWithJoystick extends Command {

    public final ElevatorSubsystem elev;
    public final DoubleSupplier joyZ;

    public MoveElevWithJoystick(ElevatorSubsystem elev, DoubleSupplier joystickZ) {
        this.elev = elev;
        addRequirements(elev);

        this.joyZ = joystickZ;
    }

    @Override
    public void execute() {
        elev.goToPosition(joyZ.getAsDouble() * 27);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
