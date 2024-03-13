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
import frc.robot.commands.Pivot.MoveToAngle;

public class PivotSubsystem extends SubsystemBase {

    private TalonFX pivotMotor;
    private DutyCycleEncoder absEncoder;
    private double lastTarget = -1;
    private int inRangeCounter = 0;
    private PIDController pivotRioPID;

    public PivotSubsystem() {
        pivotMotor = new TalonFX(Constants.Pivot.pivotId);
        pivotMotor.setInverted(true); //constant

        pivotRioPID = new PIDController(Constants.Pivot.pid.p, Constants.Pivot.pid.i, Constants.Pivot.pid.d);
        //set status frame period 
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 20.0;
        talonFXConfigs.CurrentLimits.SupplyTimeThreshold = 0.3;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 0.8;
        talonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.8;

        talonFXConfigs.Slot0.kS = 0;
        talonFXConfigs.Slot0.kV = 0;
        talonFXConfigs.Slot0.kP = Constants.Pivot.pid.p;
        talonFXConfigs.Slot0.kI = Constants.Pivot.pid.i;
        talonFXConfigs.Slot0.kD = Constants.Pivot.pid.d;
        talonFXConfigs.Slot0.kG = 0;
        //slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; config the arm sensor stuff *LOOK AT THIS* 
        
         talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        // talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        

        // talonFXConfigs.Feedback.SensorToMechanismRatio = 100; *LOOK AT THIS?*

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

        // pivotMotor.set(pivotRioPID.calculate(absEncoder.getAbsolutePosition(), lastTarget) + Constants.Pivot.pid.f * lastTarget);
    }

    public void pivotToAngle(double ticks) {
        double nTicks = ticks;
        // double nTicks = ticks * Constants.Pivot.relToAbsConversion;

        SmartDashboard.putNumber("Pivot Target (Ticks)", nTicks);
        lastTarget = nTicks;
        inRangeCounter = 0;

        pivotMotor.setControl(new PositionDutyCycle(ticks).withSlot(0));
    }
 
    public void pivotUp() {
        pivotToAngle( pivotMotor.getPosition().getValueAsDouble() + 15);
        // pivotToAngle(absEncoder.getAbsolutePosition() + 15 * Constants.Pivot.relToAbsConversion);
    }

    public boolean isAtTarget() {
        double error = lastTarget - pivotMotor.getPosition().getValueAsDouble();
        // double error = lastTarget - absEncoder.getAbsolutePosition();/
        final int timeRequired = 8;
        final double deadzone = 0.6;

        if(Math.abs(error) <= deadzone) inRangeCounter++;
        else inRangeCounter = 0;

        if(inRangeCounter >= timeRequired) return true;

        return false;
    }
}
