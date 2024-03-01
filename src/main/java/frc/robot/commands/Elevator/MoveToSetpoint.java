package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToSetpoint extends Command {

  private final ElevatorSubsystem elevatorSub;
  private final double target;

  public MoveToSetpoint(ElevatorSubsystem nElevSub, double targetPos) {
    elevatorSub = nElevSub;
    addRequirements(nElevSub);

    target = targetPos;
  }

  @Override
  public void initialize() {
    elevatorSub.goToPosition(target);
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    boolean isDone = elevatorSub.isAtTarget();

    if(isDone) {
      elevatorSub.stopElevator();
    }

    return isDone;
  }
}