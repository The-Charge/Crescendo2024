package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.Elevator.MoveElevatorToSetpoint;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class StateMachine extends Command {
    
    private final ElevatorSubsystem elevSub;
    private final PivotSubsystem pivSub;
    private final BooleanSupplier startup, pickupfloor, pickupsource, shootAmpTrap, shoothighrear, shoothighfront, shootlowfront;
    private State currentState;
    

    private enum State {
        STARTUP, //startup (inside)
        PICKUPFLOOR, // startup floor
        PICKUPSOURCE, // startup source
        SHOOTAMPTRAP, // shoot amp and trap
        SHOOTHIGHREAR, // shoot amp and trap
        SHOOTHIGHFRONT,
        SHOOTLOWFRONT
    }

    public StateMachine(ElevatorSubsystem elevSub, PivotSubsystem pivSub, BooleanSupplier startup, BooleanSupplier pickupfloor, BooleanSupplier pickupsource, BooleanSupplier shootAmpTrap, BooleanSupplier shoothighrear, BooleanSupplier shoothighfront, BooleanSupplier shootlowfront) {
        this.elevSub = elevSub;
        this.pivSub = pivSub;

        this.startup = startup;
        this.pickupfloor = pickupfloor;
        this.pickupsource = pickupsource;
        this.shootAmpTrap = shootAmpTrap;
        this.shoothighrear = shoothighrear;
        this.shoothighfront = shoothighfront;
        this.shootlowfront = shootlowfront;

        currentState = State.STARTUP;
    }

    @Override
    public void initialize() {
        State desiredState = getRequestedState();

        if(desiredState == null) return;

        switch(desiredState) {
            case STARTUP:
            gotoState1();
            currentState = State.STARTUP;
            break;

            case PICKUPFLOOR:
            gotoState2();
            currentState = State.PICKUPFLOOR;
            break;

            case PICKUPSOURCE:
            gotoState3();
            currentState = State.PICKUPSOURCE;
            break;

            case SHOOTAMPTRAP:
            gotoState4();
            currentState = State.SHOOTAMPTRAP;
            break;

            case SHOOTHIGHREAR:
            gotoState5();
            currentState = State.SHOOTHIGHREAR;
            break;

            case SHOOTHIGHFRONT:
            gotoState6();
            currentState = State.SHOOTHIGHFRONT;
            break;

            case SHOOTLOWFRONT:
            gotoState7();
            currentState = State.SHOOTLOWFRONT;
            break;
        }
    }
    @Override
    public void execute() {

    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return true;
    }

    private State getRequestedState() {
        if(startup.getAsBoolean()) return State.STARTUP;
        else if(pickupfloor.getAsBoolean()) return State.PICKUPFLOOR;
        else if(pickupsource.getAsBoolean()) return State.PICKUPSOURCE;
        else if(shootAmpTrap.getAsBoolean()) return State.SHOOTAMPTRAP;
        else if(shoothighrear.getAsBoolean()) return State.SHOOTHIGHREAR;
        else if(shoothighfront.getAsBoolean()) return State.SHOOTHIGHFRONT;
        else if(shootlowfront.getAsBoolean()) return State.SHOOTLOWFRONT;

        return null;
    }
    private void gotoState1() {
        switch(currentState) {
            case PICKUPFLOOR:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivToStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevToStartup)
                ).schedule();
                currentState = State.STARTUP;
            }
            break;

            case PICKUPSOURCE:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.elevToStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevToStartup)
                ).schedule();
                currentState = State.STARTUP;
            }
            
            break;

            case SHOOTAMPTRAP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivToStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevToStartup)
                ).schedule();
                currentState = State.STARTUP;
            }
            
            break;

            case SHOOTHIGHREAR:
                if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivToStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevToStartup)
                ).schedule();
                currentState = State.STARTUP;
            }
            
            break;

            case SHOOTHIGHFRONT:
                if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivToStartup),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevToStartup)
                ).schedule();
                currentState = State.STARTUP;
            }
            break;

            case SHOOTLOWFRONT:
                if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevToStartup),
                new MoveToAngle(pivSub, StateLocations.pivToStartup)

                ).schedule();
                currentState = State.STARTUP;
            }
            
            break;
        }
    }
    private void gotoState2() {
        switch(currentState) {
            case STARTUP:
            //example
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickupFloorPos),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloorPos)
                ).schedule();
                currentState = State.PICKUPFLOOR;
            }

            break;

            case PICKUPSOURCE:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickupFloorPos),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloorPos)
                ).schedule();
                currentState = State.PICKUPFLOOR;
            }
            
            break;

            case SHOOTAMPTRAP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickupFloorPos),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloorPos)
                ).schedule();
                currentState = State.PICKUPFLOOR;
            }
            
            break;

            case SHOOTHIGHREAR:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickupFloorPos),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloorPos)
                ).schedule();
                currentState = State.PICKUPFLOOR;
            }
            
            break;

            case SHOOTHIGHFRONT:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickupFloorPos),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloorPos)
                ).schedule();
                currentState = State.PICKUPFLOOR;
            }
            
            break;

            case SHOOTLOWFRONT:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickupFloorPos),
                new MoveToAngle(pivSub, StateLocations.pivotPickupFloorPos)

                ).schedule();
                currentState = State.PICKUPFLOOR;
            }
            
            break;
        }
    }
    private void gotoState3() {
        switch(currentState) {
            case STARTUP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickUpSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickUpSource)
                ).schedule();
                currentState = State.PICKUPSOURCE;
            }
            break;

            case PICKUPFLOOR:
                if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickUpSource),
                new MoveToAngle(pivSub, StateLocations.pivotPickUpSource)
  
                ).schedule();
                }
                currentState = State.PICKUPSOURCE;
            
            break;

            case SHOOTAMPTRAP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickUpSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickUpSource)
                ).schedule();
                currentState = State.PICKUPSOURCE;
            }
            
            break;

            case SHOOTHIGHREAR:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickUpSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickUpSource)
                ).schedule();
                currentState = State.PICKUPSOURCE;
            }
            
            break;

            case SHOOTHIGHFRONT:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotPickUpSource),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickUpSource)
                ).schedule();
                currentState = State.PICKUPSOURCE;
            }
            
            break;

            case SHOOTLOWFRONT:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevPickUpSource),
                new MoveToAngle(pivSub, StateLocations.pivotPickUpSource)
                
                ).schedule();
                currentState = State.PICKUPSOURCE;
            }
            
            break;
        }
    }
    private void gotoState4() {
        switch(currentState) {
            case STARTUP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
                ).schedule();
                currentState = State.SHOOTAMPTRAP;
            }
            break;

            case PICKUPFLOOR:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivotShootAmp)
  
                ).schedule();
                currentState = State.SHOOTAMPTRAP;
            }
            break;

            case PICKUPSOURCE:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
                ).schedule();
                currentState = State.SHOOTAMPTRAP;
            }
            
            break;

            case SHOOTHIGHREAR:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
                ).schedule();
                currentState = State.SHOOTAMPTRAP;
            }
            
            break;

            case SHOOTHIGHFRONT:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotShootAmp),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp)
                ).schedule();
                currentState = State.SHOOTAMPTRAP;
            }
            
            break;

            case SHOOTLOWFRONT:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevShootAmp),
                new MoveToAngle(pivSub, StateLocations.pivotShootAmp)
                
                ).schedule();
                currentState = State.SHOOTAMPTRAP;
            }
            
            break;
        }
    }
    private void gotoState5() {
        switch(currentState) {
            case STARTUP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
                ).schedule();
                currentState = State.SHOOTHIGHREAR;
            }
            break;

            case PICKUPFLOOR:
                if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivotHighRear)

                ).schedule();
                currentState = State.SHOOTHIGHREAR;
            }
            break;

            case PICKUPSOURCE:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
                ).schedule();
                currentState = State.SHOOTHIGHREAR;
            }
            
            break;

            case SHOOTAMPTRAP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
                ).schedule();
                currentState = State.SHOOTHIGHREAR;
            }
            
            break;

            case SHOOTHIGHFRONT:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighRear),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear)
                ).schedule();
                currentState = State.SHOOTHIGHREAR;
            }
            
            break;

            case SHOOTLOWFRONT:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighRear),
                new MoveToAngle(pivSub, StateLocations.pivotHighRear)
                ).schedule();
                currentState = State.SHOOTHIGHREAR;
            }
            
            break;
        }
    }
    private void gotoState6() {
        switch(currentState) {
            case STARTUP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
                ).schedule();
                currentState = State.SHOOTHIGHFRONT;
            }
            break;

            case PICKUPFLOOR:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront),
                new MoveToAngle(pivSub, StateLocations.pivotHighFront)

                ).schedule();
                currentState = State.SHOOTHIGHFRONT;
            }
            
            break;

            case PICKUPSOURCE:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
                ).schedule();
                currentState = State.SHOOTHIGHFRONT;
            }
            
            break;

            case SHOOTAMPTRAP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
                ).schedule();
                currentState = State.SHOOTHIGHFRONT;
            }
            
            break;

            case SHOOTHIGHREAR:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotHighFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront)
                ).schedule();
                currentState = State.SHOOTHIGHFRONT;
            }
            
            break;

            case SHOOTLOWFRONT:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevHighFront),
                new MoveToAngle(pivSub, StateLocations.pivotHighFront)
                
                ).schedule();
                currentState = State.SHOOTHIGHFRONT;
            }
            
            break;
        }
    }
    private void gotoState7() {
        switch(currentState) {
            case STARTUP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveToAngle(pivSub, StateLocations.pivotLowFront),
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront)
                ).schedule();
                currentState = State.SHOOTLOWFRONT;
            }
            break;

            case PICKUPFLOOR:
            if(startup.getAsBoolean()) {
                new SequentialCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivotLowFront)
                ).schedule();
                currentState = State.SHOOTLOWFRONT;
            }
            
            break;

            case PICKUPSOURCE:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivotLowFront)
                ).schedule();
                currentState = State.SHOOTLOWFRONT;
            }
            
            break;

            case SHOOTAMPTRAP:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivotLowFront)
                ).schedule();
                currentState = State.SHOOTLOWFRONT;
            }
            
            break;

            case SHOOTHIGHREAR:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivotLowFront)
                ).schedule();
                currentState = State.SHOOTLOWFRONT;
            }
            
            break;

            case SHOOTHIGHFRONT:
            if(startup.getAsBoolean()) {
                new ParallelCommandGroup(
                new MoveElevatorToSetpoint(elevSub, StateLocations.elevLowFront),
                new MoveToAngle(pivSub, StateLocations.pivotLowFront)
                ).schedule();
                currentState = State.SHOOTLOWFRONT;
            }
            
            break;
        }
    }
}
