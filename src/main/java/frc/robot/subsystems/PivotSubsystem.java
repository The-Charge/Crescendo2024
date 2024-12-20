package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
    
    private TalonFX pivotMotor;
    private int targetCounter;
    private double targetDeg = 0;
    private double manualPivotOverride = Constants.StateLocations.pivShootSpeakerFront;
    
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
        
        talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
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
        SmartDashboard.putNumber("Manual Pivot Override", getManualPivotOveride());
        
    }
    
    public void pivotToAngle(double deg) {
        targetDeg = Math.min(Math.max(deg, Constants.Pivot.minPosTicks * Constants.Pivot.ticksToDegConversion), Constants.Pivot.maxPosTicks * Constants.Pivot.ticksToDegConversion);
        // targetDeg = deg;
        resetTargetCounter();
        
        pivotMotor.setControl(new PositionDutyCycle(targetDeg / Constants.Pivot.ticksToDegConversion).withSlot(0));
    }
    public void pivotUp() {
        pivotToAngle(getAngle() + 20);
        System.out.println("Pivot up");
    }
    public void resetEncoder() {
        pivotMotor.setPosition(0);
    }
    
    public boolean isAtTarget() {
        double error = targetDeg - getAngle();
        SmartDashboard.putNumber("pivError", error);
        SmartDashboard.putNumber("pivTimer", targetCounter);
        
        if(Math.abs(error) <= Constants.Pivot.toleranceDeg) targetCounter++;
        else resetTargetCounter();
        
        if(targetCounter >= Constants.Pivot.toleranceTime) return true;
        
        return false;
    }
    public double getAngle() {
        return (pivotMotor.getPosition().getValueAsDouble() - Constants.Pivot.relOffset) * Constants.Pivot.ticksToDegConversion;
    }

    public double getManualPivotOveride() {
        return manualPivotOverride;
    }

    public void setManualPivotOverride(double pivotloc) {
        manualPivotOverride = pivotloc;
    }
    public boolean isInDeadzone() {
        return getAngle() > -40;
    }
    
    private void resetTargetCounter() {
        targetCounter = 0;
    }
}
