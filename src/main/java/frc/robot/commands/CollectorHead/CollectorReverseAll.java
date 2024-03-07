package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorReverseAll extends Command {

    private CollectorHeadSubsystem m_collector;
    private Timer timeout, feedTimer;

    public CollectorReverseAll(CollectorHeadSubsystem collector) {
        this.m_collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 1);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
