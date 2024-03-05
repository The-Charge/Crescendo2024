package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorZero extends Command {

    private CollectorHeadSubsystem m_collector;
    Timer timeout, feedTimer;

    public CollectorZero(CollectorHeadSubsystem head) {
        this.m_collector = head;
    }

    @Override
    public void initialize() {
        m_collector.zero();
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

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
