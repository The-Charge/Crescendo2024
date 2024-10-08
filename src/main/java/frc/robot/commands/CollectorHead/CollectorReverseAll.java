package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorReverseAll extends Command {

    private CollectorHeadSubsystem m_collector;
    private Timer timer;

    public CollectorReverseAll(CollectorHeadSubsystem collector) {
        this.m_collector = collector;
        addRequirements(collector); //do not require pivot

        timer = new Timer();
        timer.start();
    }

    @Override
    public void initialize() {
        m_collector.indexerVBus(-0.7);
        m_collector.shooterVBus(-0.5 / 3);
        m_collector.intakeVBus(-0.7);
    }
    @Override
    public void end(boolean interrupted) {
        m_collector.zero();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
