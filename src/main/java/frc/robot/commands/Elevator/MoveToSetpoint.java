package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToSetpoint extends Command {

  private final ElevatorSubsystem elevatorSub;
  private final double target;

  public MoveToSetpoint(ElevatorSubsystem elev, double targetPos) {
    elevatorSub = elev;
    addRequirements(elev);

    target = targetPos;
  }

  @Override
  public void initialize() {
    elevatorSub.goToPosition(target);
  }
  @Override
  public boolean isFinished() {
    //return elevatorSub.isAtTarget();
    return true;
  }
}
