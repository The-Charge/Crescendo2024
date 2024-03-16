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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Pivot.MoveToAngle;

public class PivotSubsystem extends SubsystemBase {

    private TalonFX pivotMotor;
    private Encoder absEncoder;
    private int inRangeCounter = 0;
    private PIDController pivotRioPID;
    private double targetDeg = -1;

    public PivotSubsystem() {
        pivotMotor = new TalonFX(Constants.Pivot.pivotId);
        pivotMotor.setInverted(true); //constant

        // pivotRioPID = new PIDController(Constants.Pivot.pid.p, Constants.Pivot.pid.i, Constants.Pivot.pid.d);
        pivotRioPID = new PIDController(0, 0, 0);
        pivotRioPID.setTolerance(0.5);
        //set status frame period 
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 20.0;
        talonFXConfigs.CurrentLimits.SupplyTimeThreshold = 0.3;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 0.8;
        talonFXConfigs.MotorOutput.PeakReverseDutyCycle = -0.8;

        // talonFXConfigs.Slot0.kS = 0;
        // talonFXConfigs.Slot0.kV = 0;
        // talonFXConfigs.Slot0.kP = Constants.Pivot.pid.p;
        // talonFXConfigs.Slot0.kI = Constants.Pivot.pid.i;
        // talonFXConfigs.Slot0.kD = Constants.Pivot.pid.d;
        // talonFXConfigs.Slot0.kG = 0;
        //talonFXConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 
        
         talonFXConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        

        // talonFXConfigs.Feedback.SensorToMechanismRatio = 100; *LOOK AT THIS?*

        pivotMotor.getConfigurator().apply(talonFXConfigs);

        absEncoder = new Encoder(Constants.Pivot.encoderId, 6);
        //absEncoder.reset();
        // absEncoder.setPositionOffset(0.2);
        //pivotMotor.setPosition((absEncoder.getAbsolutePosition() / Constants.Pivot.absTicksPerDeg + Constants.Pivot.absEncoderAngleOffset) * Constants.Pivot.ticksPerDeg);
        // SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        // pivotMotor.getConfigurator().apply(softLimits);
        // pivotMotor.setPosition(absEncoder.getAbsolutePosition() / Constants.Pivot.absRatio);

        SmartDashboard.putNumber("Piv kP", 0);
        SmartDashboard.putNumber("Piv kI", 0);
        SmartDashboard.putNumber("Piv kD", 0);
        SmartDashboard.putNumber("Piv kF", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot I (Amps)", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Position (Ticks)", pivotMotor.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("Pivot abs position (ticks)", absEncoder.getAbsolutePosition() );
        // SmartDashboard.putNumber("Pivot abs position (ticks)", absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset());
        SmartDashboard.putNumber("Pivot abs position (ticks)", absEncoder.get());
        SmartDashboard.putNumber("Pivot Position (Deg)", getAngle());
        
       

        double kF = SmartDashboard.getNumber("Piv kF", 0);

        double output = pivotRioPID.calculate(getAngle(), targetDeg) + kF * Math.cos(getAngle());
        output = Math.min(Math.max(output, -0.4), 0.4);

        SmartDashboard.putNumber("output pid", output);
        SmartDashboard.putNumber("target deg", targetDeg);
        pivotMotor.set(output);
    }

    public void pivotToAngle(double deg) {
        // double nTicks = ticks;
        SmartDashboard.putNumber("Pivot Target (Deg)", deg);
        targetDeg = deg;
        inRangeCounter = 0;

        // pivotMotor.setControl(new PositionDutyCycle(ticks).withSlot(0));
    }
 
    public void pivotUp() {
        // pivotToAngle( pivotMotor.getPosition().getValueAsDouble() + 15);
        pivotToAngle(getAngle() + 20);
    }
    public double getAngle() {
        return -(absEncoder.get() - Constants.Pivot.absOffset) * Constants.Pivot.absToDegConversion;
    }

    public boolean isAtTarget() {
        // double error = lastTarget - pivotMotor.getPosition().getValueAsDouble();
        // final double deadzone = 0.6;
        double error = targetDeg - getAngle();
        final double deadzone = 0.5;
        final int timeRequired = 8;

        if(Math.abs(error) <= deadzone) inRangeCounter++;
        else inRangeCounter = 0;

        if(inRangeCounter >= timeRequired) return true;

        return false;
    }
    public void resetEncoder() {
        absEncoder.reset();
    }
    public void setPIDCoefficients(double kP, double kI, double kD) {
        pivotRioPID.setPID(kP, kI, kD);
    }
}
