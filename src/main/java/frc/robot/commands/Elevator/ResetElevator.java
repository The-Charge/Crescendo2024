package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class ResetElevator extends Command {

    private ElevatorSubsystem elev;
    private MoveToAngle pivMove;
    private boolean hasSetSpeed = false;
    private boolean canEnd = false;

    public ResetElevator(ElevatorSubsystem elev, PivotSubsystem piv) {
        this.elev = elev;
        addRequirements(elev);

        pivMove = new MoveToAngle(piv, Constants.StateLocations.pivStartup);
    }

    @Override
    public void initialize() {
        pivMove.schedule();
    }
    @Override
    public void execute() {
        if(pivMove.isFinished() && !hasSetSpeed) {
            elev.moveAtSpeed(-0.3);
            hasSetSpeed = true;
        }
        if(elev.getLimit()) {
            elev.stopElevator();
            elev.setAsZero();
            canEnd = true;
        }
    }
    @Override
    public boolean isFinished() {
        return canEnd;
    }
}
