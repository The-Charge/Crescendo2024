package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorReverseAll extends Command {

    private CollectorHeadSubsystem m_collector;
    private Timer timer;

    public CollectorReverseAll(CollectorHeadSubsystem collector) {
        this.m_collector = collector;
        addRequirements(collector);

        timer = new Timer();
        timer.start();
    }

    @Override
    public void initialize() {
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.BACKWARD, 0.5);
        m_collector.spinShooter(CollectorHeadSubsystem.Direction.BACKWARD, 0.5);
        m_collector.spinIntake(CollectorHeadSubsystem.Direction.BACKWARD, 0.5);
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.5);
    }
}
