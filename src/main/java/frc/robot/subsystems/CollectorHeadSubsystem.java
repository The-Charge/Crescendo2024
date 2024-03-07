package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CollectorHeadSubsystem extends SubsystemBase {

    public enum Direction {
        FORWARD,
        BACKWARD
    }

    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax indexerLeft;
    private CANSparkMax indexerRight;
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private DigitalInput noteSensor2;
    private DigitalInput noteSensor1;

    private int inRangeCounter = 0;
    
    public CollectorHeadSubsystem() {
        noteSensor2 = new DigitalInput(Constants.Indexer.photosensor2Id); // stop for second one
        noteSensor1 = new DigitalInput(Constants.Indexer.photosensor1Id); //pivot first one
        //shooterLeft
        shooterLeft = new CANSparkMax(Constants.Shooter.leftId, MotorType.kBrushless);
        shooterLeft.set(0);
        shooterLeft.restoreFactoryDefaults();
        // shooterLeft.setMotorBrake(NeutralMode.Brake);
        shooterLeft.setSmartCurrentLimit(10);
        
        SparkPIDController pidControllerShooterLeft = shooterLeft.getPIDController();
        pidControllerShooterLeft.setOutputRange(-1, 1);

        pidControllerShooterLeft.setP(Constants.Shooter.pid.kP);
        pidControllerShooterLeft.setI(Constants.Shooter.pid.kI);
        pidControllerShooterLeft.setD(Constants.Shooter.pid.kD);

        pidControllerShooterLeft.setIZone(0);
        pidControllerShooterLeft.setFF(0);
        shooterLeft.burnFlash();
        //set to coast mode 

        //shooterRight
        shooterRight = new CANSparkMax(Constants.Shooter.rightId, MotorType.kBrushless);
        shooterRight.set(0);
        shooterRight.restoreFactoryDefaults();
        shooterRight.setSmartCurrentLimit(10);
        shooterRight.setInverted(true);
        
        SparkPIDController pidControllerShooterRight = shooterRight.getPIDController();
        pidControllerShooterRight.setOutputRange(-1, 1);
        pidControllerShooterRight.setP(Constants.Shooter.pid.kP);
        pidControllerShooterRight.setI(Constants.Shooter.pid.kI);
        pidControllerShooterRight.setD(Constants.Shooter.pid.kD);
        pidControllerShooterRight.setIZone(0);
        pidControllerShooterRight.setFF(0);
        shooterRight.burnFlash();

        //indexerLeft
        indexerLeft = new CANSparkMax(Constants.Indexer.leftId, MotorType.kBrushless);
        indexerLeft.set(0);
        indexerLeft.setSmartCurrentLimit(10);
        indexerLeft.restoreFactoryDefaults();
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
        indexerRight.set(0);
        indexerRight.restoreFactoryDefaults();
        indexerRight.setInverted(true);
        SparkPIDController pidControllerIndexerRight = indexerLeft.getPIDController();
        pidControllerIndexerRight.setOutputRange(-1, 1);
        indexerRight.setSmartCurrentLimit(10);

        pidControllerIndexerRight.setP(Constants.Indexer.pid.kP);
        pidControllerIndexerRight.setI(Constants.Indexer.pid.kI);
        pidControllerIndexerRight.setD(Constants.Indexer.pid.kD);

        pidControllerIndexerRight.setIZone(0);
        pidControllerIndexerRight.setFF(0);
        indexerRight.burnFlash();

        //intakeTop
        intakeTop = new CANSparkMax(Constants.Intake.topId, MotorType.kBrushless);
        intakeTop.set(0);
        intakeTop.restoreFactoryDefaults();
        intakeTop.setSmartCurrentLimit(10);
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
        intakeBottom.set(0);
        intakeBottom.restoreFactoryDefaults();
        intakeBottom.setInverted(false);
        intakeBottom.setSmartCurrentLimit(10);
        SparkPIDController pidControllerIntakeBottom = intakeBottom.getPIDController();
        pidControllerIntakeBottom.setOutputRange(-1, 1);

        pidControllerIntakeBottom.setP(Constants.Intake.pid.kP);
        pidControllerIntakeBottom.setI(Constants.Intake.pid.kI);
        pidControllerIntakeBottom.setD(Constants.Intake.pid.kD);
        
        pidControllerIntakeBottom.setIZone(0);
        pidControllerIntakeBottom.setFF(0);
        intakeBottom.burnFlash();

    }

    public void setVelocity(double speed, CANSparkMax collectorMotor) {
        collectorMotor.getPIDController().setReference(speed,CANSparkMax.ControlType.kVelocity);
    }
    
    public void resetTargetCounter() {
        inRangeCounter = 0;
    }
    public boolean shootIsATarget(double targetVel) {
        return isAtTarget(targetVel, shooterLeft, shooterRight);
    }
    private boolean isAtTarget(double targetVelocity, CANSparkMax motor1, CANSparkMax motor2) {
        final double deadband = 1000;
        final int requiredTime = 5;

        double error1 = targetVelocity - motor1.getEncoder().getVelocity();
        double error2 = targetVelocity - motor2.getEncoder().getVelocity();

        if(Math.abs(error1) <= deadband && Math.abs(error2) <= deadband) inRangeCounter++;
        else inRangeCounter = 0;

    
        SmartDashboard.putNumber("left encoder value", motor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("right encoder value", motor2.getEncoder().getVelocity());
        SmartDashboard.putBoolean(" is at target", inRangeCounter>=requiredTime);
        SmartDashboard.putNumber("in Range Counter", inRangeCounter);
        SmartDashboard.putNumber("current shooter left", shooterLeft.getOutputCurrent());
        SmartDashboard.putNumber("current shooter right", shooterRight.getOutputCurrent());
        
        if(inRangeCounter >= requiredTime) return true;
        return false;
        
    }


    public void zero() {
        intakeTop.disable();
        intakeBottom.disable();

        indexerLeft.disable();
        indexerRight.disable();

        shooterLeft.disable();
        shooterRight.disable();
    }
    public void spinIndexer(Direction dir, double speed) {
        if(dir == Direction.FORWARD) {
            indexerLeft.set(speed);
            indexerRight.set(speed);
        }
        else {
            indexerLeft.set(-speed);
            indexerRight.set(-speed);
        }
    }
    public void spinShooter(Direction dir, double speed) {
        if(dir == Direction.FORWARD) {
            shooterLeft.set(speed);
            shooterRight.set(speed);
        }
        else {
            shooterLeft.set(-speed);
            shooterRight.set(-speed);
        }
    }
    public void spinIntake(Direction dir, double speed) {
        if(dir == Direction.FORWARD) {
            intakeBottom.set(speed);
            intakeTop.set(speed);
        }
        else {
            intakeBottom.set(-speed);
            intakeTop.set(-speed);
        }
    }

    public boolean getNoteSensor1() {
        return noteSensor1.get();
    }
    public boolean getNoteSensor2() {
        return noteSensor2.get();
    }
}
