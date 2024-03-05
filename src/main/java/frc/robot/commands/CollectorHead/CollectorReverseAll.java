package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorReverseAll extends Command {

    private CollectorHeadSubsystem head;

    public CollectorReverseAll(CollectorHeadSubsystem head) {
        this.head = head;
    }

    @Override
    public void initialize() {
        head.spinShooter(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        head.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        head.spinIntake(CollectorHeadSubsystem.Direction.BACKWARD, 1);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
