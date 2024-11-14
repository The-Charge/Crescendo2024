package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.subsystems.*;

public class ClimbDown extends Command {
    
    private ElevatorSubsystem elev;
    private PivotSubsystem piv;
    private Command task = null;

    public ClimbDown(ElevatorSubsystem elev, PivotSubsystem piv) {
        this.elev = elev;
        this.piv = piv;
        addRequirements(elev, piv);
    }

    @Override
    public void initialize() {
        if(!elev.getHasClimbed()) return;

        task = new SequentialCommandGroup(
            new MoveToAngle(piv, StateLocations.pivFloor),
            new MoveToSetpoint(elev, 5)
        );
        task.schedule();
    }
    @Override
    public void end(boolean interrupted) {
        elev.setHasClimbed(false);
    }
    @Override
    public boolean isFinished() {
        return task == null ? true : task.isFinished();
    }
}
