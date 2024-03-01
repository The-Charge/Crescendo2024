package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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
    private Timer shootTimer;

    
   // public RelativeEncoder encoderLeft;
        public CollectorHeadSubsystem() {
            noteSensor2 = new DigitalInput(0);
            noteSensor1 = new DigitalInput(1);
            //shooterLeft
            shooterLeft = new CANSparkMax(Constants.Shooter.LEFTSHOOTERID, MotorType.kBrushless);
            shooterLeft.set(0);
            shooterLeft.restoreFactoryDefaults();
            SparkPIDController pidControllerShooterLeft = shooterLeft.getPIDController();
            pidControllerShooterLeft.setOutputRange(-1, 1);

            pidControllerShooterLeft.setP(Constants.Shooter.SHOOTERPID.kP);
            pidControllerShooterLeft.setI(Constants.Shooter.SHOOTERPID.kI);
            pidControllerShooterLeft.setD(Constants.Shooter.SHOOTERPID.kD);

            pidControllerShooterLeft.setIZone(0);
            pidControllerShooterLeft.setFF(0);
        
            //set to coast mode 

            //shooterRight
            shooterRight = new CANSparkMax(Constants.Shooter.RIGHTSHOOTERID, MotorType.kBrushless);
            shooterRight.set(0);
            shooterRight.restoreFactoryDefaults();
            SparkPIDController pidControllerShooterRight = shooterRight.getPIDController();
            pidControllerShooterRight.setOutputRange(-1, 1);
            pidControllerShooterRight.setP(Constants.Shooter.SHOOTERPID.kP);
            pidControllerShooterRight.setI(Constants.Shooter.SHOOTERPID.kI);
            pidControllerShooterRight.setD(Constants.Shooter.SHOOTERPID.kD);
            pidControllerShooterRight.setIZone(0);
            pidControllerShooterRight.setFF(0);

            //indexerLeft
            indexerLeft = new CANSparkMax(Constants.Indexer.LEFTINDEXERID, MotorType.kBrushless);
            indexerLeft.set(0);
            indexerLeft.restoreFactoryDefaults();
            SparkPIDController pidControllerIndexerLeft = indexerLeft.getPIDController();
            pidControllerIndexerLeft.setOutputRange(-1, 1);
            pidControllerIndexerLeft.setP(Constants.Indexer.INDEXERPID.kP);
            pidControllerIndexerLeft.setI(Constants.Indexer.INDEXERPID.kI);
            pidControllerIndexerLeft.setD(Constants.Indexer.INDEXERPID.kD);
            pidControllerIndexerLeft.setIZone(0);
            pidControllerIndexerLeft.setFF(0);

            //indexerRight
            indexerRight = new CANSparkMax(Constants.Indexer.RIGHTINDEXERID, MotorType.kBrushless);
            indexerRight.set(0);
            indexerRight.restoreFactoryDefaults();
            SparkPIDController pidControllerIndexerRight = indexerLeft.getPIDController();
            pidControllerIndexerRight.setOutputRange(-1, 1);

            pidControllerIndexerRight.setP(Constants.Indexer.INDEXERPID.kP);
            pidControllerIndexerRight.setI(Constants.Indexer.INDEXERPID.kI);
            pidControllerIndexerRight.setD(Constants.Indexer.INDEXERPID.kD);

            pidControllerIndexerRight.setIZone(0);
            pidControllerIndexerRight.setFF(0);

            //intakeTop
            intakeTop = new CANSparkMax(Constants.Intake.TOPINTAKEID, MotorType.kBrushless);
            intakeTop.set(0);
            intakeTop.restoreFactoryDefaults();
            SparkPIDController pidControllerIntakeTop = intakeTop.getPIDController();
            pidControllerIntakeTop.setOutputRange(-1, 1);

            pidControllerIntakeTop.setP(Constants.Intake.INTAKEPID.kP);
            pidControllerIntakeTop.setI(Constants.Intake.INTAKEPID.kI);
            pidControllerIntakeTop.setD(Constants.Intake.INTAKEPID.kD);

            pidControllerIntakeTop.setIZone(0);
            pidControllerIntakeTop.setFF(0);

            //intakeBottom
            intakeBottom = new CANSparkMax(Constants.Intake.BOTTOMINTAKEID, MotorType.kBrushless);
            intakeBottom.set(0);
            intakeBottom.restoreFactoryDefaults();
            SparkPIDController pidControllerIntakeBottom = intakeBottom.getPIDController();
            pidControllerIntakeBottom.setOutputRange(-1, 1);

            pidControllerIntakeBottom.setP(Constants.Intake.INTAKEPID.kP);
            pidControllerIntakeBottom.setI(Constants.Intake.INTAKEPID.kI);
            pidControllerIntakeBottom.setD(Constants.Intake.INTAKEPID.kD);
            
            pidControllerIntakeBottom.setIZone(0);
            pidControllerIntakeBottom.setFF(0);

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
        
        public void reverseIntake() {
            setVelocity(-12000, intakeTop);
            setVelocity(-12000, intakeBottom); 
            setVelocity(-12000, indexerLeft);
            setVelocity(-12000, indexerRight); 
        }

        public void intakeGround(){
            new Command() {
                @Override
                public void initialize() {
                    setVelocity(3600, intakeTop);
                    setVelocity(3600, intakeBottom);
                }
                @Override
                public void execute() {

                }
                @Override
                public void end(boolean interrupted) {
                    pivotUp();
                    collectorStates(State.INTAKESOURCE);
                }
                @Override
                public boolean isFinished() {
                    return getNoteSensor1();
                }
            }.schedule();
        }

        public void intakeSource() {
            new Command() {
                @Override
                public void initialize() {
                    setVelocity(3600, intakeTop);
                    setVelocity(3600, intakeBottom);
                }
                @Override
                public void execute() {
                    
                }
                @Override
                public void end(boolean interrupted) {
                    collectorStates(State.ZERO);
                }
                @Override
                public boolean isFinished() {
                    return getNoteSensor2();
                }
            }.schedule();
        }

        public void shoot() {
            new Command() {
                Timer timeout, feedTimer;

                @Override
                public void initialize() {
                    timeout = new Timer();
                    feedTimer = null;

                    inRangeCounter = 0;
                    setVelocity(12000, shooterLeft);
                    setVelocity(12000, shooterRight);
                }

                @Override
                public void execute() {
                    if(isAtTarget(12000, shooterLeft, shooterRight)) {
                        indexerForShoot();
                        feedTimer = new Timer();
                    }
                }

                @Override
                public void end(boolean interrupted) {
                    collectorStates(State.ZERO); 
                }

                @Override
                public boolean isFinished() {
                    return timeout.hasElapsed(12) || (feedTimer == null ? false : feedTimer.hasElapsed(1.5));
                }
            }.schedule();
        }

        public boolean isAtTarget(double targetVelocity, CANSparkMax motor1, CANSparkMax motor2) {
            final double deadband = 40;
            final int requiredTime = 20;

            double error1 = targetVelocity - motor1.getEncoder().getVelocity();
            double error2 = targetVelocity - motor2.getEncoder().getVelocity();
            if(Math.abs(error1) <= deadband && Math.abs(error2) <= deadband) inRangeCounter++;
            else inRangeCounter = 0;

            if(inRangeCounter >= requiredTime) return true;
            return false;
        }


        public void indexerForShoot() {
            setVelocity(12000, indexerLeft);
            setVelocity(12000, indexerRight);

            setVelocity(12000, intakeTop);
            setVelocity(12000, indexerRight);
        }

        public void pivotUp() {
            
        }

        public void collectorStates(State targetState) {
            switch(targetState) {
                case REVERSEINTAKE:
                    reverseIntake();
                    break;
                case INTAKESOURCE:
                    intakeSource();
                    break;
                case INTAKEGROUND:
                    intakeGround();
                case SHOOT:
                    shoot();
                    break;
                case ZERO:
                    zero();
                    break;
            }
        }
        //collector head tells robot when to pivot
        //monitor current 
        // 
        public enum State {
            REVERSEINTAKE,
            INTAKESOURCE,
            INTAKEGROUND,
            PIVOTUP,
            SHOOT,
            ZERO
        }


        public void zero() {
            intakeTop.disable();
            intakeBottom.disable();

            indexerLeft.disable();
            indexerRight.disable();

            shooterLeft.disable();
            shooterRight.disable();
        }

        private boolean getNoteSensor2() {
            return noteSensor2.get();
        }

        private boolean getNoteSensor1() {
            return noteSensor1.get();
        }
}