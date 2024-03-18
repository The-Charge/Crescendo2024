package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class ClimbUp extends Command {
    
    private ElevatorSubsystem elev;
    private PivotSubsystem piv;
    private Command task;

    public ClimbUp(ElevatorSubsystem elev, PivotSubsystem piv) {
        this.elev = elev;
        this.piv = piv;
        addRequirements(elev, piv);
    }

    @Override
    public void initialize() {
        task = new SequentialCommandGroup(
            new MoveToAngle(piv, StateLocations.pivFloor),
            new MoveToSetpoint(elev, StateLocations.elevClimb)
        );
        task.schedule();

        elev.setHasClimbed(true);
    }
    @Override
    public boolean isFinished() {
        return task.isFinished();
    }
}
