package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CollectorShoot extends Command {

    private CollectorHeadSubsystem m_collector;
    private PivotSubsystem m_pivot;
    private Timer timeout, feedTimer;
    private boolean hasStartedIndexers, hasSetTime;

    public CollectorShoot(CollectorHeadSubsystem collector, PivotSubsystem pivot) {
        this.m_collector = collector;
        addRequirements(collector); //do not reequire pivot

        this.m_pivot = pivot;
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        feedTimer = null;
        timeout.start();

        m_collector.resetTargetCounter();
        m_collector.shooterVBus(1);
        hasStartedIndexers = false;
        hasSetTime = false;
    }
    @Override
    public void execute() {
        if((m_collector.shootIsATarget() && !hasStartedIndexers) || (timeout.hasElapsed(3))) {
            m_collector.indexerVBus(1);
            m_collector.intakeVBus(1);
            feedTimer = new Timer();
            feedTimer.start();
            hasStartedIndexers = true;
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_collector.zero();
    }
    @Override
    public boolean isFinished() {
        if(hasSetTime) {
            SmartDashboard.putNumber("feed timer", feedTimer.get());
        }
        return timeout.hasElapsed(4.25) || (feedTimer == null ? false : feedTimer.hasElapsed(1.25)) || m_pivot.isInDeadzone();
    }
}
