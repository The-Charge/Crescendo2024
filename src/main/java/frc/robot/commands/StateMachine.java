package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.Elevator.MoveToSetpoint;
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
        SHOOOTSHALLOWFRONT,
        SHOOTSTEEPFRONT,
        TRAVEL
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

            case SHOOOTSHALLOWFRONT:
            task = goToShallowFront();
            currentState = State.SHOOOTSHALLOWFRONT;
            break;

            case SHOOTSTEEPFRONT:
            task = goToSteepFront();
            currentState = State.SHOOTSTEEPFRONT;
            break;

            case TRAVEL:
            task = goToTravel();
            currentState = State.TRAVEL;
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
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case PICKUPSOURCE:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.elevStartup),
                new MoveToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOOTSHALLOWFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.safeElevatorPoint),
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveToSetpoint(elevSub, StateLocations.elevStartup)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevStartup),
                new MoveToAngle(pivSub, StateLocations.pivStartup)
            );

            case TRAVEL:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivStartup),
                new MoveToSetpoint(elevSub, StateLocations.elevStartup)
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
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case PICKUPSOURCE:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOOTSHALLOWFRONT:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.safeElevatorPoint),
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            case TRAVEL:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupFloor),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupFloor)
            );

            default:
            return null;
        }
    }
    private Command goToPickupSource() {
        switch(currentState) {
            case STARTUP:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource),
                new MoveToAngle(pivSub, StateLocations.pivPickupSource)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource),
                new MoveToAngle(pivSub, StateLocations.pivPickupSource)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case SHOOOTSHALLOWFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource),
                new MoveToAngle(pivSub, StateLocations.pivPickupSource)
            );

            case TRAVEL:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivPickupSource),
                new MoveToSetpoint(elevSub, StateLocations.elevPickupSource)
            );

            default:
            return null;
        }
    }
    private Command goToShootAmpTrap() {
        switch(currentState) {
            case STARTUP:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivShootAmp)
            );

            case PICKUPFLOOR:
            return new ParallelCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivShootAmp)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case SHOOOTSHALLOWFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivShootAmp)
            );

            case TRAVEL:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShootAmp),
                new MoveToSetpoint(elevSub, StateLocations.elevShootAmp)
            );

            default:
            return null;
        }
    }
    private Command goToHighRear() {
        switch(currentState) {
            case STARTUP:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivHighRear)
            );

            case PICKUPFLOOR:
            return new ParallelCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivHighRear)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case SHOOOTSHALLOWFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivHighRear)
            );

            case TRAVEL:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivHighRear),
                new MoveToSetpoint(elevSub, StateLocations.elevHighRear)
            );

            default:
            return null;
        }
    }
    private Command goToShallowFront() {
        switch(currentState) {
            case STARTUP:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront),
                new MoveToAngle(pivSub, StateLocations.pivShallowFront)
            );

            case PICKUPFLOOR:
            return new ParallelCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront),
                new MoveToAngle(pivSub, StateLocations.pivShallowFront)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShallowFront),
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShallowFront),
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShallowFront),
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront),
                new MoveToAngle(pivSub, StateLocations.pivShallowFront)
            );

            case TRAVEL:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivShallowFront),
                new MoveToSetpoint(elevSub, StateLocations.elevShallowFront)
            );

            default:
            return null;
        }
    }
    private Command goToSteepFront() {
        switch(currentState) {
            case STARTUP:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.safeElevatorPoint),
                new MoveToAngle(pivSub, StateLocations.pivSteepFront),
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.safeElevatorPoint),
                new MoveToAngle(pivSub, StateLocations.pivSteepFront),
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront)
            );

            case PICKUPSOURCE:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivSteepFront),
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront)
            );

            case SHOOTAMPTRAP:
            return new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivSteepFront),
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront)
            );

            case SHOOTHIGHREAR:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront),
                new MoveToAngle(pivSub, StateLocations.pivSteepFront)
            );

            case SHOOOTSHALLOWFRONT:
            return new ParallelCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront),
                new MoveToAngle(pivSub, StateLocations.pivSteepFront)
            );

            case TRAVEL:
            return new ParallelCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevSteepFront),
                new MoveToAngle(pivSub, StateLocations.pivSteepFront)
            );

            default:
            return null;
        }
    }
    private Command goToTravel() {
        switch(currentState) {
            case STARTUP:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevTravel),
                new MoveToAngle(pivSub, StateLocations.pivTravel)
            );

            case PICKUPFLOOR:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevTravel),
                new MoveToAngle(pivSub, StateLocations.pivTravel)
            );

            case PICKUPSOURCE:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivTravel),
                new MoveToSetpoint(elevSub, StateLocations.elevTravel)
            );

            case SHOOTAMPTRAP:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivTravel),
                new MoveToSetpoint(elevSub, StateLocations.elevTravel)
            );

            case SHOOTHIGHREAR:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivTravel),
                new MoveToSetpoint(elevSub, StateLocations.elevTravel)
            );

            case SHOOOTSHALLOWFRONT:
            return new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivTravel),
                new MoveToSetpoint(elevSub, StateLocations.elevTravel)
            );

            case SHOOTSTEEPFRONT:
            return new SequentialCommandGroup(
                new MoveToSetpoint(elevSub, StateLocations.elevTravel),
                new MoveToAngle(pivSub, StateLocations.pivTravel)
            );

            default:
            return null;
        }
    }
}
