package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberUp extends Command {
  private final ClimbSubsystem climber;
  private final double power;

  public ClimberUp(ClimbSubsystem subsystem, double power) {
    climber = subsystem;
    addRequirements(subsystem);

    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      climber.setPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
