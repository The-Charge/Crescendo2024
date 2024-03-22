package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.Elevator.MoveToSetpoint;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class MovePivotElev extends Command {
    private ElevatorSubsystem m_elevator;
    private PivotSubsystem m_pivot;
    private double elevSetpoint;
    private double pivSetpoint;
    private Command action;
    private boolean waitForTask;
    private double x;

    public MovePivotElev(ElevatorSubsystem elev, PivotSubsystem piv, double elevTarget, double pivotSetpoint, boolean wait) {
        m_elevator = elev;
        m_pivot = piv;
        addRequirements(elev, piv);

        elevSetpoint = elevTarget;
        action = null;
        pivSetpoint = pivotSetpoint;
        waitForTask = wait;
        x=0;
    }

    public MovePivotElev(ElevatorSubsystem elev, PivotSubsystem piv, double elevTarget, double pivotSetpoint) {
        m_elevator = elev;
        m_pivot = piv;
        addRequirements(elev, piv);

        elevSetpoint = elevTarget;
        action = null;
        pivSetpoint = pivotSetpoint;
        waitForTask = false;
        x=0;
    }

    @Override
    public void initialize() {
        if(pivSetpoint == Constants.StateLocations.pivShootSpeakerCenter) {
            pivSetpoint = m_pivot.getManualPivotOveride();
        }

        if((pivSetpoint > 0 || pivSetpoint < StateLocations.pivFloor) && (elevSetpoint < StateLocations.safeElevatorPoint)) {
            //DriverStation.reportError("Invalid location for elev/piv", null);
            return;
        } 
        else if(m_elevator.getPosition() < StateLocations.safeElevatorPoint && elevSetpoint > StateLocations.safeElevatorPoint && (pivSetpoint > 0 || pivSetpoint < StateLocations.pivFloor)) {
            action = new SequentialCommandGroup(
                new MoveToSetpoint(m_elevator, elevSetpoint),
                new MoveToAngle(m_pivot, pivSetpoint)
            );
        }
        else if((m_pivot.getAngle() > 0 || m_pivot.getAngle() < StateLocations.pivFloor) && m_elevator.getPosition() > StateLocations.safeElevatorPoint && elevSetpoint < StateLocations.safeElevatorPoint) {
            action = new SequentialCommandGroup(
                new MoveToAngle(m_pivot, pivSetpoint),
                new MoveToSetpoint(m_elevator, elevSetpoint)
            );
        }
        else {
            action = new ParallelCommandGroup(
                new MoveToSetpoint(m_elevator, elevSetpoint),
                new MoveToAngle(m_pivot, pivSetpoint)
            );
        }

        action.schedule();
    }

    @Override
    public void execute() {
        //SmartDashboard.putBoolean("auto", action.isScheduled());
        x+=1;
        SmartDashboard.putNumber("auto 3", x);
    }

    @Override
    public boolean isFinished() {
        // if(!waitForTask) return true;
     
        // return action == null ? true : action.isFinished();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("auto 2", true);
    }
}
