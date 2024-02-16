package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class SpinShooter extends Command {

  private final ShooterSubsystem m_shooter;
  private CANSparkMax flywheel;
  public double speedGoal;

  public SpinShooter(ShooterSubsystem subsystem, double speed) {
    m_shooter = subsystem;
    addRequirements(subsystem);
    speedGoal = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setVelocity(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double target = speedGoal.getAsDouble();

    // flywheel.set(target);
    //m_shooter.setVelocity(); //rpm!!

    // if(Math.abs(target) <= 0.1) target = 0;


    // m_shooter.setVelocity(target * 5000);
    m_shooter.setVelocity(speedGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  m_shooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
