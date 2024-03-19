package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class ResetElevator extends Command {

    private ElevatorSubsystem elev;
    private MoveToAngle pivMove;
    private boolean hasCoasted = false;

    public ResetElevator(ElevatorSubsystem elev, PivotSubsystem piv) {
        this.elev = elev;
        addRequirements(elev, piv);

        pivMove = new MoveToAngle(piv, Constants.StateLocations.pivSource);
    }

    @Override
    public void initialize() {
        pivMove.schedule();
    }
    @Override
    public void execute() {
        SmartDashboard.putBoolean("resetPivIsFinished", pivMove.isFinished());
        if(pivMove.isFinished() && !hasCoasted) {
            elev.coast();
            elev.stopElevator();
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
