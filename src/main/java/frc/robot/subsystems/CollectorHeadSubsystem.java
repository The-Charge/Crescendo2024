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

    private CANSparkMax shooterLeft;
    private CANSparkMax shooterRight;
    private CANSparkMax indexerLeft;
    private CANSparkMax indexerRight;
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private DigitalInput noteSensor2;
    private DigitalInput noteSensor1;

    private int inRangeCounter = 0;

    
   // public RelativeEncoder encoderLeft;
        public CollectorHeadSubsystem() {
            noteSensor2 = new DigitalInput(9); // initial second one
            noteSensor1 = new DigitalInput(8); //initial first one
            //shooterLeft
            shooterLeft = new CANSparkMax(Constants.Shooter.LEFTSHOOTERID, MotorType.kBrushless);
            shooterLeft.set(0);
            shooterLeft.restoreFactoryDefaults();
           // shooterLeft.setMotorBrake(NeutralMode.Brake);
           shooterLeft.setSmartCurrentLimit(10);
           
            SparkPIDController pidControllerShooterLeft = shooterLeft.getPIDController();
            pidControllerShooterLeft.setOutputRange(-1, 1);

            pidControllerShooterLeft.setP(Constants.Shooter.SHOOTERPID.kP);
            pidControllerShooterLeft.setI(Constants.Shooter.SHOOTERPID.kI);
            pidControllerShooterLeft.setD(Constants.Shooter.SHOOTERPID.kD);

            pidControllerShooterLeft.setIZone(0);
            pidControllerShooterLeft.setFF(0);
            shooterLeft.burnFlash();
            //set to coast mode 

            //shooterRight
            shooterRight = new CANSparkMax(Constants.Shooter.RIGHTSHOOTERID, MotorType.kBrushless);
            shooterRight.set(0);
            shooterRight.restoreFactoryDefaults();
            shooterRight.setSmartCurrentLimit(10);
            shooterRight.setInverted(true);
            
            SparkPIDController pidControllerShooterRight = shooterRight.getPIDController();
            pidControllerShooterRight.setOutputRange(-1, 1);
            pidControllerShooterRight.setP(Constants.Shooter.SHOOTERPID.kP);
            pidControllerShooterRight.setI(Constants.Shooter.SHOOTERPID.kI);
            pidControllerShooterRight.setD(Constants.Shooter.SHOOTERPID.kD);
            pidControllerShooterRight.setIZone(0);
            pidControllerShooterRight.setFF(0);
            shooterRight.burnFlash();

            //indexerLeft
            indexerLeft = new CANSparkMax(Constants.Indexer.LEFTINDEXERID, MotorType.kBrushless);
            indexerLeft.set(0);
            indexerLeft.setSmartCurrentLimit(10);
            indexerLeft.restoreFactoryDefaults();
            SparkPIDController pidControllerIndexerLeft = indexerLeft.getPIDController();
            pidControllerIndexerLeft.setOutputRange(-1, 1);
            pidControllerIndexerLeft.setP(Constants.Indexer.INDEXERPID.kP);
            pidControllerIndexerLeft.setI(Constants.Indexer.INDEXERPID.kI);
            pidControllerIndexerLeft.setD(Constants.Indexer.INDEXERPID.kD);
            pidControllerIndexerLeft.setIZone(0);
            pidControllerIndexerLeft.setFF(0);
            indexerLeft.burnFlash();

            //indexerRight
            indexerRight = new CANSparkMax(Constants.Indexer.RIGHTINDEXERID, MotorType.kBrushless);
            indexerRight.set(0);
            indexerRight.restoreFactoryDefaults();
            indexerRight.setInverted(true);
            SparkPIDController pidControllerIndexerRight = indexerLeft.getPIDController();
            pidControllerIndexerRight.setOutputRange(-1, 1);
            indexerRight.setSmartCurrentLimit(10);

            pidControllerIndexerRight.setP(Constants.Indexer.INDEXERPID.kP);
            pidControllerIndexerRight.setI(Constants.Indexer.INDEXERPID.kI);
            pidControllerIndexerRight.setD(Constants.Indexer.INDEXERPID.kD);

            pidControllerIndexerRight.setIZone(0);
            pidControllerIndexerRight.setFF(0);
            indexerRight.burnFlash();

            //intakeTop
            intakeTop = new CANSparkMax(Constants.Intake.TOPINTAKEID, MotorType.kBrushless);
            intakeTop.set(0);
            intakeTop.restoreFactoryDefaults();
            intakeTop.setSmartCurrentLimit(10);
            SparkPIDController pidControllerIntakeTop = intakeTop.getPIDController();
            pidControllerIntakeTop.setOutputRange(-1, 1);


            pidControllerIntakeTop.setP(Constants.Intake.INTAKEPID.kP);
            pidControllerIntakeTop.setI(Constants.Intake.INTAKEPID.kI);
            pidControllerIntakeTop.setD(Constants.Intake.INTAKEPID.kD);

            pidControllerIntakeTop.setIZone(0);
            pidControllerIntakeTop.setFF(0);
            intakeTop.burnFlash();

            //intakeBottom
            intakeBottom = new CANSparkMax(Constants.Intake.BOTTOMINTAKEID, MotorType.kBrushless);
            intakeBottom.set(0);
            intakeBottom.restoreFactoryDefaults();
            intakeBottom.setInverted(false);
            intakeBottom.setSmartCurrentLimit(10);
            SparkPIDController pidControllerIntakeBottom = intakeBottom.getPIDController();
            pidControllerIntakeBottom.setOutputRange(-1, 1);

            pidControllerIntakeBottom.setP(Constants.Intake.INTAKEPID.kP);
            pidControllerIntakeBottom.setI(Constants.Intake.INTAKEPID.kI);
            pidControllerIntakeBottom.setD(Constants.Intake.INTAKEPID.kD);
            
            pidControllerIntakeBottom.setIZone(0);
            pidControllerIntakeBottom.setFF(0);
            intakeBottom.burnFlash();

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

        public CANSparkMax getFlywheel(CANSparkMax flywheel) {
            return flywheel;
        }

        public void setVelocity(double speed, CANSparkMax collectorMotor) {
            collectorMotor.getPIDController().setReference(speed,CANSparkMax.ControlType.kVelocity);
        }
        
        public void reverseAll() {
            intakeTop.set(-.5);
            intakeBottom.set(-.5);
            indexerLeft.set(-.5);
            indexerRight.set(-.5);
            shooterLeft.set(-.5);
            shooterRight.set(-.5);
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

        public boolean getNoteSensor2() {
            return noteSensor2.get();
        }

        public boolean getNoteSensor1() {
            return noteSensor1.get();
        }


        public void spinIndexer(Direction dir, double speed) {
            if(dir == Direction.FORWARD) {
                indexerLeft.set(speed);
                indexerRight.set(speed);
            }
            else {
                indexerLeft.set(speed);
                indexerRight.set(speed);
            }
        }

        public void spinShooter(Direction dir, double speed) {
            if(dir == Direction.FORWARD) {
                indexerLeft.set(speed);
                indexerRight.set(speed);
            }
            else {
                indexerLeft.set(speed);
                indexerRight.set(speed);
            }
        }

        public void spinIntake(Direction dir, double speed) {
            if(dir == Direction.FORWARD) {
                indexerLeft.set(speed);
                indexerRight.set(speed);
            }
            else {
                indexerLeft.set(speed);
                indexerRight.set(speed);
            }
        }

        public enum Direction {
            FORWARD,
            BACKWARD
        }


}