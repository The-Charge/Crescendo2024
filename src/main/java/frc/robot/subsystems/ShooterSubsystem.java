package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax flywheel;
    private SparkPIDController pidController;
    private double lp, li, ld, ls;

    public ShooterSubsystem() {
        flywheel = new CANSparkMax(Constants.Shooter.leftShooterId, MotorType.kBrushless);
        flywheel.restoreFactoryDefaults();
        flywheel.setIdleMode(IdleMode.kBrake);

        flywheel.restoreFactoryDefaults();
        
        pidController = flywheel.getPIDController();
        pidController.setOutputRange(-1, 1);
        configurePIDFactors(0, 0, 0);
    }

    @Override
    public void periodic() {}
    @Override
    public void simulationPeriodic() {}

    public void setFlywheelPower(double power) {
        flywheel.set(power);
    }
    public void configurePIDFactors(double kp, double ki, double kd) {
        pidController.setP(kp);
        pidController.setI(ki);
        pidController.setD(kd);

        lp = kp;
        li = ki;
        ld = kd;
    }
    public double getVelocity() {
        return flywheel.getEncoder().getVelocity();
    }

    public void spinFlywheelInit() {
        SmartDashboard.putNumber("Velocity", getVelocity());
        SmartDashboard.putNumber("kP", 0); //readonly
        SmartDashboard.putNumber("kI", 0); //readonly
        SmartDashboard.putNumber("kD", 0); //readonly
        SmartDashboard.putNumber("Setpoint", 0); //readonly
    }
    public void spinFlywheelExecute() {
        SmartDashboard.putNumber("Velocity", getVelocity());

        double kp = SmartDashboard.getNumber("kP", 0);
        double ki = SmartDashboard.getNumber("kI", 0);
        double kd = SmartDashboard.getNumber("kD", 0);
        if(lp != kp || li != ki || ld != kd) configurePIDFactors(kp, ki, kd);

        double setpoint = SmartDashboard.getNumber("Setpoint", 0);
        if(setpoint != ls) {
            pidController.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
            ls = setpoint;
        }
    }
}