package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX leftDriver, rightDriver;
    private MotionMagicVoltage magicRequest;
    private Relay brake;
    private double lastTarget = -1;
    private int inRangeCounter = 0;

    public ElevatorSubsystem() {
        leftDriver = new TalonFX(Constants.Elevator.leftDriverId);
        rightDriver = new TalonFX(Constants.Elevator.rightDriverId);

        rightDriver.setControl(new Follower(leftDriver.getDeviceID(), true));

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

        leftDriver.setNeutralMode(NeutralModeValue.Brake);
        rightDriver.setNeutralMode(NeutralModeValue.Brake);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = Constants.Elevator.maxPos;
        softLimits.ReverseSoftLimitThreshold = Constants.Elevator.minPos;
        leftDriver.getConfigurator().apply(softLimits);

        leftDriver.getConfigurator().apply(slotConfigs);
        magicRequest = new MotionMagicVoltage(Constants.Elevator.homePos).withSlot(0);

        brake = new Relay(Constants.Elevator.brakeId);
        brake.set(Relay.Value.kOff);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator pos", leftDriver.getPosition().getValueAsDouble());
    }
    @Override
    public void simulationPeriodic() {}

    public void goToPosition(double target) {
        //clamp target within a safe range
        lastTarget = Math.min(Math.max(target, Constants.Elevator.minPos), Constants.Elevator.maxPos);
        inRangeCounter = 0;

        magicRequest.withPosition(lastTarget);

        //make sure to disable brake before moving for safety
        disableBrake();
        leftDriver.setControl(magicRequest);
    }
    public void enableBrake() {
        brake.set(Relay.Value.kForward);
    }
    public void disableBrake() {
        brake.set(Relay.Value.kReverse);
    }
    public void stopElevator() {
        leftDriver.disable();
    }
    public void driveByPower(double power) {
        power = Math.min(Math.max(power, 0), 1);

        if(power != 0) disableBrake();
        leftDriver.set(power);
        if(power == 0) enableBrake();
    }

    public boolean isAtTarget() {
        double error = lastTarget - leftDriver.getPosition().getValueAsDouble();

        if(Math.abs(error) <= Constants.Elevator.rangeSize) inRangeCounter++;

        if(inRangeCounter >= Constants.Elevator.rangeTime) return true;

        return false;
    }
}