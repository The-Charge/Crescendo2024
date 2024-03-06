package frc.robot.commands;

import java.util.function.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class PivotElevator extends Command {

    private final ElevatorSubsystem elev;
    private final PivotSubsystem pivot;
    private final IntSupplier machineState;
    
    public PivotElevator( ElevatorSubsystem elev, PivotSubsystem piv, IntSupplier machineState) {
        this.elev = elev;
        this.pivot = piv;
        this.machineState = machineState;
    }

    @Override
    public void execute() {
        if(machineState.getAsInt() >= 0 && machineState.getAsInt() < StateMachine.State.values().length) {
            new StateMachine(elev, pivot, StateMachine.State.values()[machineState.getAsInt()]).schedule();
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
