package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorZero extends Command {

    private CollectorHeadSubsystem m_collector;
    private Timer timeout, feedTimer;

    public CollectorZero(CollectorHeadSubsystem collector) {
        this.m_collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        m_collector.zero();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
