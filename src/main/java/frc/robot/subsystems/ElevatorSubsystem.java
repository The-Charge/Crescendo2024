package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private TalonFX elevMotor;
    private int targetCounter;
    private double currentTarget = 0;
    private boolean hasClimbed = false;
    
    public ElevatorSubsystem() {
        elevMotor = new TalonFX(Constants.Elevator.elevatorId);
        
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = Constants.Elevator.maxVBus;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -Constants.Elevator.maxVBus;
        motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        
        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.Elevator.currentLimit;
        motorConfig.CurrentLimits.SupplyTimeThreshold = 0.3;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = Constants.Elevator.pid.p;
        slotConfigs.kI = Constants.Elevator.pid.i;
        slotConfigs.kD = Constants.Elevator.pid.d;
        slotConfigs.kG = Constants.Elevator.kG;
        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;
        elevMotor.getConfigurator().apply(slotConfigs);
        
        elevMotor.getConfigurator().apply(motorConfig);
        
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = Constants.Elevator.maxPos;
        softLimits.ReverseSoftLimitThreshold = Constants.Elevator.minPos;
        elevMotor.getConfigurator().apply(softLimits);
        
        resetTargetCounter();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator I (Amps)", elevMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position (Ticks)", elevMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position (In)", getPosition());
        SmartDashboard.putNumber("Elevator Target (Inches)", currentTarget);
    }
    
    public void goToPosition(double inches) {
        currentTarget = Math.min(Math.max(inches, Constants.Elevator.minPos * Constants.Elevator.tickToInchConversion), Constants.Elevator.maxPos * Constants.Elevator.tickToInchConversion);
        resetTargetCounter();
        
        PositionDutyCycle request = new PositionDutyCycle(currentTarget / Constants.Elevator.tickToInchConversion);
        request.Slot = 0;
        elevMotor.setControl(request);
    }
    public void stopElevator() {
        elevMotor.set(0);
    }
    public void moveAtSpeed(double speed) {
        elevMotor.set(Math.min(Math.max(speed, -1), 1));
    }
    public void brake() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.PeakForwardDutyCycle = Constants.Elevator.maxVBus;
        config.MotorOutput.PeakReverseDutyCycle = -Constants.Elevator.maxVBus;
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        
        elevMotor.getConfigurator().apply(config.MotorOutput);
    }
    public void coast() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.PeakForwardDutyCycle = Constants.Elevator.maxVBus;
        config.MotorOutput.PeakReverseDutyCycle = -Constants.Elevator.maxVBus;
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        
        elevMotor.getConfigurator().apply(config.MotorOutput);
    }
    public void setHasClimbed(boolean hasClimbed) {
        this.hasClimbed = hasClimbed;
    }
    public boolean getHasClimbed() {return this.hasClimbed;}

    public double getPosition() {
        return elevMotor.getPosition().getValueAsDouble() * Constants.Elevator.tickToInchConversion;
    }
    public void setAsZero() {
        elevMotor.setPosition(0);
    }
    public boolean isAtTarget() {
        double error = currentTarget - getPosition();
        
        if(Math.abs(error) <= Constants.Elevator.rangeSize) targetCounter++;
        else resetTargetCounter();
        
        if(targetCounter >= Constants.Elevator.rangeTime) return true;
        
        return false;
    }
    
    private void resetTargetCounter() {
        targetCounter = 0;
    }
}
