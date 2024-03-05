package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;


public class CollectorIntakeSource extends Command {

  private final CollectorHeadSubsystem m_collector;

  public CollectorIntakeSource(CollectorHeadSubsystem subsystem) {
    m_collector = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_collector.spinIntake(CollectorHeadSubsystem.Direction.FORWARD, 1);
    m_collector.spinIndexer(CollectorHeadSubsystem.Direction.FORWARD, 1);
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