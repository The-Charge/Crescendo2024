package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax flywheel;

    public ShooterSubsystem() {
        flywheel = new CANSparkMax(6, MotorType.kBrushless);
        flywheel.restoreFactoryDefaults();
        
        SparkPIDController pidController = flywheel.getPIDController();
    
        RelativeEncoder encoder = flywheel.getEncoder();

        pidController.setOutputRange(-1, 1);
        pidController.setP(0.01);
        pidController.setI(0);
        pidController.setD(0);
        pidController.setIZone(0);
        pidController.setFF(0);
        
        
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

    public CANSparkMax getFlywheel() {
        return flywheel;
    }

    public void setVelocity(double speed) {
        flywheel.getPIDController().setReference(speed,CANSparkMax.ControlType.kVelocity);

    }
}