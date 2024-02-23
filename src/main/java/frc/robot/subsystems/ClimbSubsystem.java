package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private TalonFX motor;

    public ClimbSubsystem() {
        motor = new TalonFX(Constants.Climber.climberId);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
    // This method will be called once per scheduler run when in simulation
    @Override
    public void simulationPeriodic() {

    }

}