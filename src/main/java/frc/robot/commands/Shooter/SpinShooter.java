package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class SpinShooter extends Command {

  private final ShooterSubsystem m_shooter;
  private CANSparkMax flywheel;

  public SpinShooter(ShooterSubsystem subsystem) {
    m_shooter = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel = m_shooter.getFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double target = 0.3;

    // flywheel.set(target);
    m_shooter.setVelocity(120); //rpm!!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_shooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
