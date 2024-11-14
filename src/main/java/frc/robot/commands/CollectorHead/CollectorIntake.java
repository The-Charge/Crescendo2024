package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StateLocations;
import frc.robot.subsystems.*;

public class CollectorIntake extends Command {
    
    private CollectorHeadSubsystem collector;
    private PivotSubsystem pivot;
    private Command task;

    public CollectorIntake(CollectorHeadSubsystem collector, PivotSubsystem pivot) {
        this.collector = collector;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        if(Math.abs(pivot.getAngle() - StateLocations.pivFloor) <= 2) {
            System.out.println("ground");
            task = new CollectorIntakeGround(collector, pivot);
        }
        else {
            System.out.println("source");
            task = new CollectorIntakeSource(collector);
        }
        task.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
