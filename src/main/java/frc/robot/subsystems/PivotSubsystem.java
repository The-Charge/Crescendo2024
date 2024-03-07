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
        CurrentLimitsConfigs CurrentLimits = talonFXConfigs.CurrentLimits;
        CurrentLimits.StatorCurrentLimit = 20.0;
        CurrentLimits.SupplyTimeThreshold = 0.3;
        CurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Pivot.kS;
        slot0Configs.kV = Constants.Pivot.kV;
        slot0Configs.kP = Constants.Pivot.pid.p;
        slot0Configs.kI = Constants.Pivot.pid.i;
        slot0Configs.kD = Constants.Pivot.pid.d;
        //slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; config the arm sensor stuff
        
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        pivotMotor.getConfigurator().apply(talonFXConfigs);

        absEncoder = new DutyCycleEncoder(Constants.Pivot.encoderId);
        pivotMotor.setPosition((absEncoder.getAbsolutePosition() / Constants.Pivot.absTicksPerDeg + Constants.Pivot.absEncoderAngleOffset) * Constants.Pivot.ticksPerDeg);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("REL Position (degrees)", pivotMotor.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("ABS Position + offset (degrees)", absEncoder.getAbsolutePosition() * Constants.Pivot.absTicksPerDeg + Constants.Pivot.absEncoderAngleOffset);
    }

    public void pivotToAngle(double angle) {
    //    if(Math.abs(pivotMotor.getPosition().getValueAsDouble() - angle) <0.2) {
    //         pivotMotor.set(0.0);
    //         atSetpoint = true;
    //    }
        double nAngle = Math.min(Math.max(angle, Constants.Pivot.minPos), Constants.Pivot.maxPos);
        
        pivotMotor.setControl(new PositionDutyCycle(nAngle * Constants.Pivot.ticksPerDeg / Constants.Pivot.gearRat).withSlot(0));
        //atSetpoint = false;
        //pivotMotor.set(angle);
    }
    // public boolean atSetpoint() {
    //     return atSetpoint;
    // }
    public void pivotUp() {
        pivotToAngle(getCurrentAngle() + 20);
    }
    
    private double getCurrentAngle() {
        return pivotMotor.getPosition().getValueAsDouble() / Constants.Pivot.ticksPerDeg * Constants.Pivot.gearRat; //getPosition() is in rotations
    }

}