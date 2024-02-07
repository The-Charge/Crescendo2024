package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//sparkmax
public class SenseNote extends Command {

  private final IndexerSubsystem m_indexer;

//pivot, elevator: motion magic
//pivot, elevator, climber: position
//intake, indexer, shooter: speed control
  public SenseNote(IndexerSubsystem subsystem) {
    m_indexer = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean noteValue = m_indexer.getNoteSensorValue();
    SmartDashboard.putBoolean("Note Sensor", noteValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}