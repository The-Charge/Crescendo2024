package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class CollectorIntakeGround extends Command {
    private final CollectorHeadSubsystem m_collector;
    private final PivotSubsystem m_pivot;

    public CollectorIntakeGround(CollectorHeadSubsystem subsystem, PivotSubsystem pivot) {
    m_collector = subsystem;
    m_pivot = pivot;
    addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_collector.spinIntake(CollectorHeadSubsystem.Direction.FORWARD, 0.3);
    }
    @Override
    public void execute() {
        if(m_collector.getNoteSensor1()) {
            m_pivot.pivotUp();
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_collector.zero();
    }
        @Override
        public boolean isFinished() {
            return m_collector.getNoteSensor2();
        }
}
