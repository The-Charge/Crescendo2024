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

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX driver;
    private MotionMagicVoltage magicRequest;
    private double lastTarget = -1;
    private int inRangeCounter = 0;

    public ElevatorSubsystem() {
        driver = new TalonFX(Constants.Elevator.driverId);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = Constants.Elevator.pid.p;
        slotConfigs.kI = Constants.Elevator.pid.i;
        slotConfigs.kD = Constants.Elevator.pid.d;
        slotConfigs.kG = Constants.Elevator.pid.f; // forward value to hold the elevator in place
        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs magicConfig = motorConfig.MotionMagic;
        magicConfig.MotionMagicCruiseVelocity = Constants.Elevator.magicCruisVelocity;
        magicConfig.MotionMagicAcceleration = Constants.Elevator.magicAcceleration;
        magicConfig.MotionMagicJerk = Constants.Elevator.magicJerk;

        driver.setNeutralMode(NeutralModeValue.Brake);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = Constants.Elevator.maxPos;
        softLimits.ReverseSoftLimitThreshold = Constants.Elevator.minPos;
        driver.getConfigurator().apply(softLimits);

        driver.getConfigurator().apply(slotConfigs);
        magicRequest = new MotionMagicVoltage(0).withSlot(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator pos", driver.getPosition().getValueAsDouble());
    }
    @Override
    public void simulationPeriodic() {}

    //NOTE: target is in inches from the bottom of the elevators range
    public void goToPosition(double target) {
        //clamp target within a safe range
        lastTarget = Math.min(Math.max(target * Constants.Elevator.ticksPerInch, Constants.Elevator.minPos), Constants.Elevator.maxPos);
        inRangeCounter = 0;

        magicRequest.withPosition(lastTarget);
        driver.setControl(magicRequest);
    }
    public void stopElevator() {
        driver.setControl(new NeutralOut());
    }
    public boolean isAtTarget() {
        double error = lastTarget - driver.getPosition().getValueAsDouble();

        if(Math.abs(error) <= Constants.Elevator.rangeSize) inRangeCounter++;
        else inRangeCounter = 0;

        if(inRangeCounter >= Constants.Elevator.rangeTime) return true;

        return false;
    }
}