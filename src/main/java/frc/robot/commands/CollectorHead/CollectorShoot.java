package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorShoot extends Command {

    private CollectorHeadSubsystem m_collector;
    private Timer timeout, feedTimer, delay;
    private boolean hasReachedSpeed;

    public CollectorShoot(CollectorHeadSubsystem collector) {
        this.m_collector = collector;
        addRequirements(collector);
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        timeout.start();
        feedTimer = null;
        delay = null;

        m_collector.resetTargetCounter();
        m_collector.spinShooter(CollectorHeadSubsystem.Direction.FORWARD, 1);
        hasReachedSpeed = false;
    }
    @Override
    public void execute() {
        if((m_collector.shootIsATarget(10000, 10700) && !hasReachedSpeed)) {
            hasReachedSpeed = true;
            
            feedTimer = new Timer();
            feedTimer.start();
            delay = new Timer();
            delay.start();
        }
        if(delay.hasElapsed(0.5)) {
            m_collector.spinIndexer(CollectorHeadSubsystem.Direction.FORWARD, 1);
            m_collector.spinIntake(CollectorHeadSubsystem.Direction.FORWARD, 1);
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_collector.zero();
    }
    @Override
    public boolean isFinished() {
        if(hasReachedSpeed) {
            SmartDashboard.putNumber("feed timer", feedTimer.get());
        }
        return timeout.hasElapsed(4) || (feedTimer == null ? false : feedTimer.hasElapsed(1.25 + 0.5));
    }
}
