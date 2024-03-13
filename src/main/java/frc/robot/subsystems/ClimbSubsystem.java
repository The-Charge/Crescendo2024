package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climber;

    public ClimbSubsystem() {
        climber = new TalonFX(Constants.Climber.climberId);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.9;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.9;
        motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyTimeThreshold = 0.3;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climber.getConfigurator().apply(motorConfig);
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

    public void setPower(double power) {
        climber.set(power);
    }
}
