package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

    private TalonFX pivotMotor;
    private DutyCycleEncoder absEncoder;

    public PivotSubsystem() {
        pivotMotor = new TalonFX(Constants.Pivot.pivotId);
        pivotMotor.setInverted(true); //constant

        //set status frame period 
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 20.0;
        talonFXConfigs.CurrentLimits.SupplyTimeThreshold = 0.3;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 0.5;
        talonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.5;

        talonFXConfigs.Slot0.kS = Constants.Pivot.kS;
        talonFXConfigs.Slot0.kV = Constants.Pivot.kV;
        talonFXConfigs.Slot0.kP = Constants.Pivot.pid.p;
        talonFXConfigs.Slot0.kI = Constants.Pivot.pid.i;
        talonFXConfigs.Slot0.kD = Constants.Pivot.pid.d;
        talonFXConfigs.Slot0.kG = Constants.Pivot.kG;
        //slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; config the arm sensor stuff
        
        talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast); 
        

        // talonFXConfigs.Feedback.SensorToMechanismRatio = 100;

        pivotMotor.getConfigurator().apply(talonFXConfigs);

        absEncoder = new DutyCycleEncoder(Constants.Pivot.encoderId);
        //pivotMotor.setPosition((absEncoder.getAbsolutePosition() / Constants.Pivot.absTicksPerDeg + Constants.Pivot.absEncoderAngleOffset) * Constants.Pivot.ticksPerDeg);
        // SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        // pivotMotor.getConfigurator().apply(softLimits);
        // pivotMotor.setPosition(absEncoder.getAbsolutePosition() / Constants.Pivot.absRatio);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot I (Amps)", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Position (Ticks)", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot abs position (ticks)", absEncoder.getAbsolutePosition());
    }

    public void pivotToAngle(double ticks) {
    //    if(Math.abs(pivotMotor.getPosition().getValueAsDouble() - angle) <0.2) {
    //         pivotMotor.set(0.0);
    //         atSetpoint = true;
    //    }
        // double nTicks = Math.min(Math.max(ticks, Constants.Pivot.minPos), Constants.Pivot.maxPos);
        double nTicks = ticks;
        SmartDashboard.putNumber("Pivot Target (Ticks)", nTicks);
        
        pivotMotor.setControl(new PositionDutyCycle(nTicks).withSlot(0));
        //atSetpoint = false;
        //pivotMotor.set(angle);
    }
    // public boolean atSetpoint() {
    //     return atSetpoint;
    // }
    public void pivotUp() {
        pivotToAngle( pivotMotor.getPosition().getValueAsDouble() + 2);
    }
    
    private double getCurrentAngle() {
        return pivotMotor.getPosition().getValueAsDouble() / Constants.Pivot.ticksPerDeg * Constants.Pivot.gearRat; //getPosition() is in rotations
    }
}
