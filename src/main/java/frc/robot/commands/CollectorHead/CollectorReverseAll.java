package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorReverseAll extends Command {

<<<<<<< Updated upstream
    private CollectorHeadSubsystem head;
=======
    private CollectorHeadSubsystem m_collector;
    Timer timeout, feedTimer;
>>>>>>> Stashed changes

    public CollectorReverseAll(CollectorHeadSubsystem head) {
        this.m_collector = head;
    }

    @Override
    public void initialize() {
<<<<<<< Updated upstream
        head.spinShooter(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        head.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        head.spinIntake(CollectorHeadSubsystem.Direction.BACKWARD, 1);
=======
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
>>>>>>> Stashed changes
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
