package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

//2: climb, 2: elevator, 2: pivot, 19 motors
public class MoveToAngle extends Command {
  public double angleGoal;
  private final PivotSubsystem m_pivot;
  private double kFeedForward;

  public MoveToAngle(PivotSubsystem subsystem, double angle) {
    m_pivot = subsystem;
    angleGoal = angle;
    addRequirements(subsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    m_pivot.pivotToAngle(angleGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    
  }
}