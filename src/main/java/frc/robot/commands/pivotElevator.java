package frc.robot.commands;

import java.util.function.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class pivotElevator extends Command {

    private final ElevatorSubsystem elev;
    private final PivotSubsystem pivot;

    private final IntSupplier machineState;
    

    public pivotElevator( ElevatorSubsystem elev, PivotSubsystem piv, IntSupplier machineState) {
        this.elev = elev;
        this.pivot = piv;
        this.machineState = machineState;
    }

    @Override
    public void initialize() {
 
    }
    @Override
    public void execute() {
        if(machineState.getAsInt() >= 0 && machineState.getAsInt() < StateMachine.State.values().length) {
            new StateMachine(elev, pivot, StateMachine.State.values()[machineState.getAsInt()]).schedule();
        }
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
