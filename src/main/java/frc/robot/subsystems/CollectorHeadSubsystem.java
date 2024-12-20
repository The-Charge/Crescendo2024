package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;

public class CollectorHeadSubsystem extends SubsystemBase {
    
    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax indexerLeft;
    private CANSparkMax indexerRight;
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private DigitalInput noteSensor2;
    private DigitalInput noteSensor1;
    // private int targetCounter;
    
    public CollectorHeadSubsystem() {
        noteSensor2 = new DigitalInput(Constants.Indexer.photosensor2Id);
        noteSensor1 = new DigitalInput(Constants.Indexer.photosensor1Id);
        
        //shooterLeft
        shooterLeft = new CANSparkMax(Constants.Shooter.leftId, MotorType.kBrushless);
        shooterLeft.restoreFactoryDefaults();
        shooterLeft.set(0);
        shooterLeft.setIdleMode(IdleMode.kCoast);
        shooterLeft.setSmartCurrentLimit(Constants.Shooter.maxCurrent);
        
        SparkPIDController pidControllerShooterLeft = shooterLeft.getPIDController();
        pidControllerShooterLeft.setOutputRange(-1, 1);
        
        pidControllerShooterLeft.setP(Constants.Shooter.pid.p);
        pidControllerShooterLeft.setI(Constants.Shooter.pid.i);
        pidControllerShooterLeft.setD(Constants.Shooter.pid.d);
        pidControllerShooterLeft.setIZone(0);
        pidControllerShooterLeft.setFF(Constants.Shooter.pid.f);
        shooterLeft.burnFlash();
        
        //shooterRight
        shooterRight = new CANSparkMax(Constants.Shooter.rightId, MotorType.kBrushless);
        shooterRight.restoreFactoryDefaults();
        shooterRight.set(0);
        shooterRight.setIdleMode(IdleMode.kCoast);
        shooterRight.setSmartCurrentLimit(Constants.Shooter.maxCurrent);
        shooterRight.setInverted(true);
        
        SparkPIDController pidControllerShooterRight = shooterRight.getPIDController();
        pidControllerShooterRight.setOutputRange(-1, 1);
        pidControllerShooterRight.setP(Constants.Shooter.pid.p);
        pidControllerShooterRight.setI(Constants.Shooter.pid.i);
        pidControllerShooterRight.setD(Constants.Shooter.pid.d);
        pidControllerShooterRight.setIZone(0);
        pidControllerShooterRight.setFF(Constants.Shooter.pid.f);
        shooterRight.burnFlash();
        
        //indexerLeft
        indexerLeft = new CANSparkMax(Constants.Indexer.leftId, MotorType.kBrushless);
        indexerLeft.restoreFactoryDefaults();
        indexerLeft.set(0);
        indexerLeft.setIdleMode(IdleMode.kCoast);
        indexerLeft.setSmartCurrentLimit(Constants.Indexer.maxCurrent);
        
        SparkPIDController pidControllerIndexerLeft = indexerLeft.getPIDController();
        pidControllerIndexerLeft.setOutputRange(-1, 1);
        pidControllerIndexerLeft.setP(Constants.Indexer.pid.kP);
        pidControllerIndexerLeft.setI(Constants.Indexer.pid.kI);
        pidControllerIndexerLeft.setD(Constants.Indexer.pid.kD);
        pidControllerIndexerLeft.setIZone(0);
        pidControllerIndexerLeft.setFF(0);
        indexerLeft.burnFlash();
        
        //indexerRight
        indexerRight = new CANSparkMax(Constants.Indexer.rightId, MotorType.kBrushless);
        indexerRight.restoreFactoryDefaults();
        indexerRight.set(0);
        indexerRight.setIdleMode(IdleMode.kCoast);
        indexerRight.setSmartCurrentLimit(Constants.Indexer.maxCurrent);
        indexerRight.setInverted(true);
        
        SparkPIDController pidControllerIndexerRight = indexerLeft.getPIDController();
        pidControllerIndexerRight.setOutputRange(-1, 1);
        pidControllerIndexerRight.setP(Constants.Indexer.pid.kP);
        pidControllerIndexerRight.setI(Constants.Indexer.pid.kI);
        pidControllerIndexerRight.setD(Constants.Indexer.pid.kD);
        pidControllerIndexerRight.setIZone(0);
        pidControllerIndexerRight.setFF(0);
        indexerRight.burnFlash();
        
        //intakeTop
        intakeTop = new CANSparkMax(Constants.Intake.topId, MotorType.kBrushless);
        intakeTop.restoreFactoryDefaults();
        intakeTop.set(0);
        intakeTop.setIdleMode(IdleMode.kBrake);
        intakeTop.setSmartCurrentLimit(Constants.Intake.maxCurrent);
        
        SparkPIDController pidControllerIntakeTop = intakeTop.getPIDController();
        pidControllerIntakeTop.setOutputRange(-1, 1);
        pidControllerIntakeTop.setP(Constants.Intake.pid.kP);
        pidControllerIntakeTop.setI(Constants.Intake.pid.kI);
        pidControllerIntakeTop.setD(Constants.Intake.pid.kD);
        pidControllerIntakeTop.setIZone(0);
        pidControllerIntakeTop.setFF(0);
        intakeTop.burnFlash();
        
        //intakeBottom
        intakeBottom = new CANSparkMax(Constants.Intake.bottomId, MotorType.kBrushless);
        intakeBottom.restoreFactoryDefaults();
        intakeBottom.set(0);
        intakeBottom.setIdleMode(IdleMode.kBrake);
        intakeBottom.setSmartCurrentLimit(Constants.Intake.maxCurrent);
        
        SparkPIDController pidControllerIntakeBottom = intakeBottom.getPIDController();
        pidControllerIntakeBottom.setOutputRange(-1, 1);
        pidControllerIntakeBottom.setP(Constants.Intake.pid.kP);
        pidControllerIntakeBottom.setI(Constants.Intake.pid.kI);
        pidControllerIntakeBottom.setD(Constants.Intake.pid.kD);
        pidControllerIntakeBottom.setIZone(0);
        pidControllerIntakeBottom.setFF(0);
        intakeBottom.burnFlash();
        
        resetTargetCounter();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("L Shooter I (Amps)", shooterLeft.getOutputCurrent());
        SmartDashboard.putNumber("R Shooter I (Amps)", shooterRight.getOutputCurrent());
        SmartDashboard.putNumber("L Indexer I (Amps)", indexerLeft.getOutputCurrent());
        SmartDashboard.putNumber("R Indexer I (Amps)", indexerRight.getOutputCurrent());
        SmartDashboard.putNumber("T Intake I (Amps)", intakeTop.getOutputCurrent());
        SmartDashboard.putNumber("B Intake I (Amps)", intakeBottom.getOutputCurrent());
        
        SmartDashboard.putNumber("L Shooter Vel (RPM)", shooterLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("R Shooter Vel (RPM)", shooterRight.getEncoder().getVelocity());
        SmartDashboard.putNumber("L Indexer Vel (RPM)", indexerLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("R Indexer Vel (RPM)", indexerRight.getEncoder().getVelocity());
        SmartDashboard.putNumber("T Intake Vel (RPM)", intakeTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("B Intake Vel (RPM)", intakeBottom.getEncoder().getVelocity());

        SmartDashboard.putBoolean("Photosensor 2", noteSensor2.get());

        if(Robot.getContainer().getPivotSubsystem().isInDeadzone() && (shooterLeft.getEncoder().getVelocity() > 100 || shooterRight.getEncoder().getVelocity() > 100)) zero();
    }
    
    public void zero() {
        intakeTop.disable();
        intakeBottom.disable();
        
        indexerLeft.disable();
        indexerRight.disable();
        
        shooterLeft.disable();
        shooterRight.disable();
    }
    public void shooterVBus(double percent) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone()) return;

        shooterLeft.set(percent);
        shooterRight.set(percent);
    }
    public void indexerVBus(double percent) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone() && percent >= 0.9) return;

        indexerLeft.set(percent);
        indexerRight.set(percent);
    }
    public void intakeVBus(double percent) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone() && percent >= 0.9) return;
        
        intakeBottom.set(percent + 0.05);
        intakeTop.set(percent);
    }
    public void shooterVel(double vel) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone()) return;

        setTargetVel(vel, shooterLeft);
        setTargetVel(vel, shooterRight);
    }
    public void indexerVel(double vel) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone() && vel >= 9000) return;

        setTargetVel(vel, indexerLeft);
        setTargetVel(vel, indexerRight);
    }
    public void intakeVel(double vel) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone() && vel >= 9000) return;

        setTargetVel(vel, intakeBottom);
        setTargetVel(vel, intakeTop);
    }
    private void setTargetVel(double speed, CANSparkMax motor) {
        if(Robot.getContainer().getPivotSubsystem().isInDeadzone()) return;
        
        motor.getPIDController().setReference(speed,CANSparkMax.ControlType.kVelocity);
    }
    
    public boolean shootIsATarget() {
        // return isAtTarget(Constants.Shooter.leftVelTarget, Constants.Shooter.rightVelTarget, Constants.Shooter.velDeadband, Constants.Shooter.velDeadbandTime, shooterLeft, shooterRight);
        return Robot.getContainer().getPivotSubsystem().isInDeadzone() || shooterLeft.getEncoder().getVelocity() > Constants.Shooter.leftVelTarget && shooterRight.getEncoder().getVelocity() > Constants.Shooter.rightVelTarget;
    }
    // private boolean isAtTarget(double target1, double target2, double deadband, double requiredTime, CANSparkMax motor1, CANSparkMax motor2) {
    //     double error1 = target1 - motor1.getEncoder().getVelocity();
    //     double error2 = target2 - motor2.getEncoder().getVelocity();
        
    //     if(Math.abs(error1) <= deadband && Math.abs(error2) <= deadband) targetCounter++;
    //     else resetTargetCounter();
        
    //     SmartDashboard.putNumber("L encoder value", motor1.getEncoder().getVelocity());
    //     SmartDashboard.putNumber("R encoder value", motor2.getEncoder().getVelocity());
    //     SmartDashboard.putBoolean("Has Reached Target", targetCounter >= requiredTime);
    //     SmartDashboard.putNumber("Deadband Timer", targetCounter);
        
    //     if(targetCounter >= requiredTime) return true;
    //     return false;
    // }
    public void resetTargetCounter() {
        // targetCounter = 0;
    }
    
    public boolean getNoteSensor1() {
        return noteSensor1.get();
    }
    public boolean getNoteSensor2() {
        return noteSensor2.get();
    }
}
