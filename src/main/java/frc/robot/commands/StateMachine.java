package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.Elevator.MoveElevatorToSetpoint;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class StateMachine extends Command {
    
    private static State currentState = State.STARTUP;

    private final ElevatorSubsystem elevSub;
    private final PivotSubsystem pivSub;
    private final State targetState;
    private final boolean waitForTask;
    
    private Command task;

    public enum State {
        STARTUP, //startup (inside)
        PICKUPFLOOR, // startup floor
        PICKUPSOURCE, // startup source
        SHOOTAMPTRAP, // shoot amp and trap
        SHOOTHIGHREAR, // shoot amp and trap
        SHOOTHIGHFRONT,
        SHOOTLOWFRONT
    }

    public StateMachine(ElevatorSubsystem elevSub, PivotSubsystem pivSub, State targetState) {
        this.elevSub = elevSub;
        this.pivSub = pivSub;

        this.targetState = targetState;
        this.waitForTask = false;
        this.task = null;
    }
    public StateMachine(ElevatorSubsystem elevSub, PivotSubsystem pivSub, State targetState, boolean waitForTask) {
        this.elevSub = elevSub;
        this.pivSub = pivSub;

        this.targetState = targetState;
        this.waitForTask = waitForTask;
        this.task = null;
    }

    @Override
    public void initialize() {
        if(targetState == currentState) return;

        switch(targetState) {
            case STARTUP:
            task = goToStartup();
            currentState = State.STARTUP;
            break;

            case PICKUPFLOOR:
            task = goToPickupFloor();
            currentState = State.PICKUPFLOOR;
            break;

            case PICKUPSOURCE:
            task = goToPickupSource();
            currentState = State.PICKUPSOURCE;
            break;

            case SHOOTAMPTRAP:
            task = goToShootAmpTrap();
            currentState = State.SHOOTAMPTRAP;
            break;

            case SHOOTHIGHREAR:
            task = goToHighRear();
            currentState = State.SHOOTHIGHREAR;
            break;

            case SHOOTHIGHFRONT:
            task = goToHighFront();
            currentState = State.SHOOTHIGHFRONT;
            break;

            case SHOOTLOWFRONT:
            task = goToLowFront();
            currentState = State.SHOOTLOWFRONT;
            break;
        }

        if(task != null) task.schedule();
    }
    @Override
    public void execute() {

    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        if(!waitForTask) return true;

        return task == null ? true : task.isFinished();
    }

    private Command goToStartup() {
        switch(currentState) {
            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.elevStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTHIGHFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTLOWFRONT:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevStartup),
                new MoveToAngle(pivSub, StateLocations.pivStartup)
            );

            default:
            return null;
        }
    }
    private Command goToPickupFloor() {
        switch(currentState) {
            case STARTUP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTHIGHREAR:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTHIGHFRONT:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTLOWFRONT:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloor),
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor)
            );

            default:
            return null;
        }
    }
    private Command goToPickupSource() {
        switch(currentState) {
            case STARTUP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupSource),
                new MoveToAngle(pivSub, StateLocations.pivPickupSource)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case SHOOTHIGHFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case SHOOTLOWFRONT:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupSource),
                new MoveToAngle(pivSub, StateLocations.pivPickupSource)
            );

            default:
            return null;
        }
    }
    private Command goToShootAmpTrap() {
        switch(currentState) {
            case STARTUP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivShootAmp)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case SHOOTHIGHFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case SHOOTLOWFRONT:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivShootAmp)
            );

            default:
            return null;
        }
    }
    private Command goToHighRear() {
        switch(currentState) {
            case STARTUP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivHighRear)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case SHOOTHIGHFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case SHOOTLOWFRONT:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivHighRear)
            );

            default:
            return null;
        }
    }
    private Command goToHighFront() {
        switch(currentState) {
            case STARTUP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront),
                new MoveToAngle(pivSub, StateLocations.pivHighFront)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
            );

            case SHOOTLOWFRONT:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront),
                new MoveToAngle(pivSub, StateLocations.pivHighFront)
            );

            default:
            return null;
        }
    }
    private Command goToLowFront() {
        switch(currentState) {
            case STARTUP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivLowFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivLowFront)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivLowFront)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivLowFront)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivLowFront)
            );

            case SHOOTHIGHFRONT:
            return new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivLowFront)
            );

            default:
            return null;
        }
    }
}
