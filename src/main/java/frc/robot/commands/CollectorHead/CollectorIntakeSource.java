package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;


public class CollectorIntakeSource extends Command {
    
    private final CollectorHeadSubsystem m_collector;
    private final Timer timeout;
    
    public CollectorIntakeSource(CollectorHeadSubsystem collector) {
      m_collector = collector;
      addRequirements(collector);

      timeout = new Timer();
      timeout.start();
    }
    
    @Override
    public void initialize() {
        m_collector.spinIntake(CollectorHeadSubsystem.Direction.FORWARD, 0.3);
        m_collector.spinIndexer(CollectorHeadSubsystem.Direction.FORWARD, 0.3);
    }
    @Override
    public void end(boolean interrupted) {
        m_collector.zero();
    }
    @Override
    public boolean isFinished() {
      return m_collector.getNoteSensor2() || timeout.hasElapsed(5);
    }
}
