package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorBrake extends Command {

  private final ElevatorSubsystem elevatorSub;
  private final boolean state;

  public SetElevatorBrake(ElevatorSubsystem nElevSub, boolean enabled) {
    elevatorSub = nElevSub;
    addRequirements(nElevSub);

    state = enabled;
  }

  @Override
  public void initialize() {
    if(state) elevatorSub.enableBrake();
    else elevatorSub.disableBrake();
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}