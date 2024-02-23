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
    private final BooleanSupplier state1, state2, state3, state4, state5, state6, state7;
    private State currentState;

    private enum State {
        STATE1,
        STATE2,
        STATE3,
        STATE4,
        STATE5,
        STATE6,
        STATE7
    }

    public StateMachine(ElevatorSubsystem elevSub, PivotSubsystem pivSub, BooleanSupplier state1, BooleanSupplier state2, BooleanSupplier state3, BooleanSupplier state4, BooleanSupplier state5, BooleanSupplier state6, BooleanSupplier state7) {
        this.elevSub = elevSub;
        this.pivSub = pivSub;

        this.state1 = state1;
        this.state2 = state2;
        this.state3 = state3;
        this.state4 = state4;
        this.state5 = state5;
        this.state6 = state6;
        this.state7 = state7;
        currentState = State.STATE1;
    }

    @Override
    public void initialize() {
        State desiredState = getRequestedState();

        if(desiredState == null) return;

        switch(desiredState) {
            case STATE1:
            gotoState1();
            currentState = State.STATE1;
            break;

            case STATE2:
            gotoState2();
            currentState = State.STATE2;
            break;

            case STATE3:
            gotoState3();
            currentState = State.STATE3;
            break;

            case STATE4:
            gotoState4();
            currentState = State.STATE4;
            break;

            case STATE5:
            gotoState5();
            currentState = State.STATE5;
            break;

            case STATE6:
            gotoState6();
            currentState = State.STATE6;
            break;

            case STATE7:
            gotoState7();
            currentState = State.STATE7;
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
        if(state1.getAsBoolean()) return State.STATE1;
        else if(state2.getAsBoolean()) return State.STATE2;
        else if(state3.getAsBoolean()) return State.STATE3;
        else if(state4.getAsBoolean()) return State.STATE4;
        else if(state5.getAsBoolean()) return State.STATE5;
        else if(state6.getAsBoolean()) return State.STATE6;
        else if(state7.getAsBoolean()) return State.STATE7;

        return null;
    }
    private void gotoState1() {
        switch(currentState) {
            case STATE2:
            
            break;

            case STATE3:
            
            break;

            case STATE4:
            
            break;

            case STATE5:
            
            break;

            case STATE6:
            
            break;

            case STATE7:
            
            break;
        }
    }
    private void gotoState2() {
        switch(currentState) {
            case STATE1:
            //example
            // if(vY.getAsDouble() > 0.1) {
            //     new SequentialCommandGroup(
            //     new MoveElevatorToSetpoint(elevSub, StateLocations.elevPos1),
            //     new MoveToAngle(pivSub, StateLocations.pivPos1)
            //     ).schedule();
            // }
            break;

            case STATE3:
            
            break;

            case STATE4:
            
            break;

            case STATE5:
            
            break;

            case STATE6:
            
            break;

            case STATE7:
            
            break;
        }
    }
    private void gotoState3() {
        switch(currentState) {
            case STATE1:
            
            break;

            case STATE2:
            
            break;

            case STATE4:
            
            break;

            case STATE5:
            
            break;

            case STATE6:
            
            break;

            case STATE7:
            
            break;
        }
    }
    private void gotoState4() {
        switch(currentState) {
            case STATE1:
            
            break;

            case STATE2:
            
            break;

            case STATE3:
            
            break;

            case STATE5:
            
            break;

            case STATE6:
            
            break;

            case STATE7:
            
            break;
        }
    }
    private void gotoState5() {
        switch(currentState) {
            case STATE1:
            
            break;

            case STATE2:
            
            break;

            case STATE3:
            
            break;

            case STATE4:
            
            break;

            case STATE6:
            
            break;

            case STATE7:
            
            break;
        }
    }
    private void gotoState6() {
        switch(currentState) {
            case STATE1:
            
            break;

            case STATE2:
            
            break;

            case STATE3:
            
            break;

            case STATE4:
            
            break;

            case STATE5:
            
            break;

            case STATE7:
            
            break;
        }
    }
    private void gotoState7() {
        switch(currentState) {
            case STATE1:
            
            break;

            case STATE2:
            
            break;

            case STATE3:
            
            break;

            case STATE4:
            
            break;

            case STATE5:
            
            break;

            case STATE6:
            
            break;
        }
    }
}
