package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class MoveToAngle extends Command {

  public double angleGoal;
  private final PivotSubsystem m_pivot;

  public MoveToAngle(PivotSubsystem subsystem, double angle) {
    m_pivot = subsystem;
    addRequirements(subsystem);
    
    angleGoal = angle;
  }

  @Override
  public void execute() {
    m_pivot.pivotToAngle(angleGoal);
  }
  @Override
  public boolean isFinished() {
    return true;
  }
}
