package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;


public class CollectorIntakeSource extends Command {
    
    private final CollectorHeadSubsystem m_collector;
    
    public CollectorIntakeSource(CollectorHeadSubsystem collector) {
      m_collector = collector;
      addRequirements(collector);
    }
    
    @Override
    public void initialize() {
        m_collector.intakeVBus(0.3);
        m_collector.indexerVBus(0.3);
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
