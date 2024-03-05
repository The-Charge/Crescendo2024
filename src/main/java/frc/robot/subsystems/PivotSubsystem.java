package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

    private PIDController PivotPIDController; 
    private TalonFX pivotMotor;
    private boolean atSetpoint;
    private MotionMagicVoltage request;
    private DutyCycleEncoder encoder;

    public PivotSubsystem() {

        //PivotPIDController = new PIDController(Constants.Auton.TranslationPID.p, Constants.Auton.TranslationPID.d, Constants.Auton.TranslationPID.i);
        pivotMotor = new TalonFX(Constants.Pivot.PivotId);
        pivotMotor.setInverted(true); //constant


        //set status frame period 
        var talonFXConfigs = new TalonFXConfiguration();

        CurrentLimitsConfigs currentLimits = talonFXConfigs.CurrentLimits;
        currentLimits.StatorCurrentLimit = 40.3;
        currentLimits.SupplyTimeThreshold = 0.3;
        currentLimits.StatorCurrentLimitEnable = true;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.Pivot.pivotkS;
        slot0Configs.kV = Constants.Pivot.pivotkV;
        slot0Configs.kP = Constants.Pivot.pivotPID.p;
        slot0Configs.kI = Constants.Pivot.pivotPID.i;
        slot0Configs.kD = Constants.Pivot.pivotPID.d;
        //slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; config the arm sensor stuff
        

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;
       
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        pivotMotor.getConfigurator().apply(talonFXConfigs);
        request = new MotionMagicVoltage(0).withSlot(0);

        encoder = new DutyCycleEncoder(Constants.Pivot.encoderId);
    }


    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("encoder pivot value", pivotMotor.getPosition().getValueAsDouble());
        
    }

    // This method will be called once per scheduler run when in simulation
    @Override
    public void simulationPeriodic() {

    }

    public void pivotToAngle(double angle) {
    //    if(Math.abs(pivotMotor.getPosition().getValueAsDouble() - angle) <0.2) {
    //         pivotMotor.set(0.0);
    //         atSetpoint = true;
    //    } 
        pivotMotor.setControl(request.withPosition(angle));
        //atSetpoint = false;
        //pivotMotor.set(angle);
    }

    // public boolean atSetpoint() {
    //     return atSetpoint;
    // }

}