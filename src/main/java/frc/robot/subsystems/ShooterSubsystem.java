package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax flywheel;
public RelativeEncoder encoder;
    public ShooterSubsystem() {
        flywheel = new CANSparkMax(6, MotorType.kBrushless);
        flywheel.restoreFactoryDefaults();
        flywheel.setSmartCurrentLimit(1);
        flywheel.setSecondaryCurrentLimit(1);
        
        SparkPIDController pidController = flywheel.getPIDController();
    
        encoder = flywheel.getEncoder();

        pidController.setOutputRange(-1, 1);
        pidController.setP(0.0001);
        pidController.setI(0.0000011);
        pidController.setD(9.999999974752427e-7);
        pidController.setIZone(0);
        pidController.setFF(0);
        
        
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter encoder value", encoder.getPosition());

        SmartDashboard.putNumber("Current (A)", flywheel.getOutputCurrent());
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