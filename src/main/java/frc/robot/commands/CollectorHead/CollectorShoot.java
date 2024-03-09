package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorShoot extends Command {

    private CollectorHeadSubsystem m_collector;
    private Timer timeout, feedTimer;
    private boolean hasStartedIndexers, hasSetTime;

    public CollectorShoot(CollectorHeadSubsystem collector) {
        this.m_collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        feedTimer = null;
        timeout.start();

        m_collector.resetTargetCounter();
        m_collector.spinShooter(CollectorHeadSubsystem.Direction.FORWARD, 1);
        hasStartedIndexers = false;
        hasSetTime = false;
    }
    @Override
    public void execute() {
        if(m_collector.shootIsATarget(10000, 10700) && !hasStartedIndexers) {
            m_collector.spinIndexer(CollectorHeadSubsystem.Direction.FORWARD, 1);
            m_collector.spinIntake(CollectorHeadSubsystem.Direction.FORWARD, 1);
            hasStartedIndexers = true;
        }
        if(!m_collector.getNoteSensor1() && !m_collector.getNoteSensor2() && !hasSetTime) {
            feedTimer = new Timer();
            feedTimer.start();
            hasSetTime = true;
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
        return timeout.hasElapsed(4) || (feedTimer == null ? false : feedTimer.hasElapsed(1.25));
    }
}
