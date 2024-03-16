package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CollectorReverseAll extends Command {

    private CollectorHeadSubsystem m_collector;
    private PivotSubsystem m_pivot;
    private Timer timer;

    public CollectorReverseAll(CollectorHeadSubsystem collector, PivotSubsystem pivot) {
        this.m_collector = collector;
        addRequirements(collector); //do not require pivot

        this.m_pivot = pivot;

        timer = new Timer();
        timer.start();
    }

    @Override
    public void initialize() {
        m_collector.indexerVBus(0.5);
        m_collector.shooterVBus(0.5);
        m_collector.intakeVBus(0.5);
    }
    @Override
    public void end(boolean interrupted) {
        // m_collector.zero();
    }
    @Override
    public boolean isFinished() {
        return m_pivot.isInDeadzone() || m_pivot.isInDeadzone();
    }
}
