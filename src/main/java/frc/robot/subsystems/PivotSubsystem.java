package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Pivot.MoveToAngle;

public class PivotSubsystem extends SubsystemBase {
    
    private TalonFX pivotMotor;
    private int targetCounter;
    private double targetDeg = 0;
    
    public PivotSubsystem() {
        pivotMotor = new TalonFX(Constants.Pivot.pivotId);
        pivotMotor.setInverted(Constants.Pivot.invertMotor);
        
        //set status frame period 
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = Constants.Pivot.maxCurrent;
        talonFXConfigs.CurrentLimits.SupplyTimeThreshold = 0.3;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        talonFXConfigs.MotorOutput.PeakForwardDutyCycle = Constants.Pivot.maxVBus;
        talonFXConfigs.MotorOutput.PeakReverseDutyCycle = -Constants.Pivot.maxVBus;
        
        talonFXConfigs.Slot0.kS = 0;
        talonFXConfigs.Slot0.kV = 0;
        talonFXConfigs.Slot0.kP = Constants.Pivot.pid.p;
        talonFXConfigs.Slot0.kI = Constants.Pivot.pid.i;
        talonFXConfigs.Slot0.kD = Constants.Pivot.pid.d;
        talonFXConfigs.Slot0.kG = 0;
        //talonFXConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 
        
        talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        
        // talonFXConfigs.Feedback.SensorToMechanismRatio = 100; *LOOK AT THIS?*
        
        pivotMotor.getConfigurator().apply(talonFXConfigs);
        
        // SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        // softLimits.ForwardSoftLimitEnable = true;
        // softLimits.ReverseSoftLimitEnable = true;
        // softLimits.ForwardSoftLimitThreshold = Constants.Pivot.maxPosTicks;
        // softLimits.ReverseSoftLimitThreshold = Constants.Pivot.minPosTicks;
        // pivotMotor.getConfigurator().apply(softLimits);
        
        resetTargetCounter();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot I (Amps)", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Position (Ticks)", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Position (Deg)", getAngle());
        SmartDashboard.putNumber("Pivot Target (Deg)", targetDeg);
    }
    
    public void pivotToAngle(double deg) {
        // targetDeg = Math.min(Math.max(deg, Constants.Pivot.minPosTicks * Constants.Pivot.ticksToDegConversion), Constants.Pivot.maxPosTicks * Constants.Pivot.ticksToDegConversion);
        targetDeg = deg;
        resetTargetCounter();
        
        pivotMotor.setControl(new PositionDutyCycle(deg / Constants.Pivot.ticksToDegConversion).withSlot(0));
    }
    public void pivotUp() {
        pivotToAngle(getAngle() + 20);
    }
    public void resetEncoder() {
        pivotMotor.setPosition(0);
    }
    
    public boolean isAtTarget() {
        double error = targetDeg - getAngle();
        
        if(Math.abs(error) <= Constants.Pivot.toleranceDeg) targetCounter++;
        else resetTargetCounter();
        
        if(targetCounter >= Constants.Pivot.toleranceTime) return true;
        
        return false;
    }
    public double getAngle() {
        return (pivotMotor.getPosition().getValueAsDouble() - Constants.Pivot.relOffset) * Constants.Pivot.ticksToDegConversion;
    }
    public boolean isInDeadzone() {
        return getAngle() > Constants.Pivot.noActionStart && getAngle() < Constants.Pivot.noActionEnd;
    }
    
    private void resetTargetCounter() {
        targetCounter = 0;
    }
}
