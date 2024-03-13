package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.Elevator.MoveToSetpoint;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX driver;
    private double lastTarget = -1;
    private int inRangeCounter = 0;

    public ElevatorSubsystem() {
        driver = new TalonFX(Constants.Elevator.elevatorId);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.9;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.9;
        motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyTimeThreshold = 0.3;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = Constants.Elevator.pid.p;
        slotConfigs.kI = Constants.Elevator.pid.i;
        slotConfigs.kD = Constants.Elevator.pid.d;
        slotConfigs.kG = Constants.Elevator.kG;
        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;
        driver.getConfigurator().apply(slotConfigs);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = Constants.Elevator.maxPos;
        softLimits.ReverseSoftLimitThreshold = Constants.Elevator.minPos;
        driver.getConfigurator().apply(softLimits);
        driver.getConfigurator().apply(motorConfig);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position (Ticks)", driver.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator I (Amps)", driver.getStatorCurrent().getValueAsDouble());
    }

    //NOTE: target is in inches from the bottom of the elevators range
    public void goToPosition(double target) {
        //clamp target within a safe range
        lastTarget = Math.min(Math.max(target * Constants.Elevator.ticksPerInch, Constants.Elevator.minPos), Constants.Elevator.maxPos);
        inRangeCounter = 0;

        SmartDashboard.putNumber("Elevator Target (Ticks)", lastTarget);

        PositionDutyCycle request = new PositionDutyCycle(lastTarget);
        request.Slot = 0;
        driver.setControl(request);
    }
    public void stopElevator() {
        driver.set(0);
    }
    public boolean isAtTarget() {
        double error = lastTarget - driver.getPosition().getValueAsDouble();

        if(Math.abs(error) <= Constants.Elevator.rangeSize * Constants.Elevator.ticksPerInch) inRangeCounter++;
        else inRangeCounter = 0;

        if(inRangeCounter >= Constants.Elevator.rangeTime) return true;

        return false;
    }
    public double elevPos() {
        return driver.getPosition().getValueAsDouble();
    }
    public void moveAtSpeed(double speed) {
        driver.set(Math.min(Math.max(speed, -1), 1));
    }
    public void setAsZero() {
        driver.setPosition(0); //might work
    }
    public void coast() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.PeakForwardDutyCycle = 0.6;
        config.MotorOutput.PeakReverseDutyCycle = -0.6;
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        driver.getConfigurator().apply(config.MotorOutput);
    }
    public void brake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.PeakForwardDutyCycle = 0.6;
        config.MotorOutput.PeakReverseDutyCycle = -0.6;
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        driver.getConfigurator().apply(config.MotorOutput);
    }
}
