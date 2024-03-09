package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class ResetElevator extends Command {

    private ElevatorSubsystem elev;
    private MoveToAngle pivMove;
    private boolean hasCoasted = false;
    private boolean canEnd = false;

    public ResetElevator(ElevatorSubsystem elev, PivotSubsystem piv) {
        this.elev = elev;
        addRequirements(elev);

        pivMove = new MoveToAngle(piv, Constants.StateLocations.pivPickupSource);
    }

    @Override
    public void initialize() {
        pivMove.schedule();
    }
    @Override
    public void execute() {
        if(pivMove.isFinished() && !hasCoasted) {
            elev.coast();
            elev.moveAtSpeed(0);
            hasCoasted = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        elev.setAsZero();
        elev.brake();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}