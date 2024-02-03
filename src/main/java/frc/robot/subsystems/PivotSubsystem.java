package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.jni.CtreJniWrapper;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

public class PivotSubsystem extends SubsystemBase {
    private PIDController PivotPIDController;
    private TalonFX pivotMotor;

    public PivotSubsystem() {

        //PivotPIDController = new PIDController(Constants.Auton.TranslationPID.p, Constants.Auton.TranslationPID.d, Constants.Auton.TranslationPID.i);
        pivotMotor = new TalonFX(Constants.Pivot.PivotId);
        pivotMotor.setInverted(false);


        //set status frame period 
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Pivot.pivotkS;
        slot0Configs.kV = Constants.Pivot.pivotkV;
        slot0Configs.kP = Constants.Pivot.pivotPID.p;
        slot0Configs.kI = Constants.Pivot.pivotPID.i;
        slot0Configs.kD = Constants.Pivot.pivotPID.d;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;
       

        pivotMotor.getConfigurator().apply(talonFXConfigs);
    }


    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    // This method will be called once per scheduler run when in simulation
    @Override
    public void simulationPeriodic() {

    }

    public void pivotToAngle(double angle) {
        var request = new MotionMagicVoltage(0).withSlot(0);
        pivotMotor.setControl(request.withPosition(10));
        //pivotMotor.set(angle);
    }

}