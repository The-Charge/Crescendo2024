package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooter extends Command {

  private final ShooterSubsystem m_shooter;

  public SpinShooter(ShooterSubsystem subsystem) {
    m_shooter = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_shooter.spinFlywheelInit();
  }
  @Override
  public void execute() {
    m_shooter.spinFlywheelExecute();
  }
  @Override
  public void end(boolean interrupted) {
    m_shooter.setFlywheelPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}