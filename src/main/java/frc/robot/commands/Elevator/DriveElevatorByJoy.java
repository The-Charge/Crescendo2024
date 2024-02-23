package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.subsystems.ElevatorSubsystem;

public class DriveElevatorByJoy extends Command {

  private final ElevatorSubsystem elevatorSub;
  private CommandJoystick joy;

  public DriveElevatorByJoy(ElevatorSubsystem nElevSub, CommandJoystick nJoy) {
    elevatorSub = nElevSub;
    addRequirements(nElevSub);

    joy = nJoy;
  }

  @Override
  public void initialize() {
    elevatorSub.driveByPower(Helpers.formatJoyInput(-joy.getY()));
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