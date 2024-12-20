// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBox;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.StateLocations;
import frc.robot.commands.MovePivotElev;
import frc.robot.commands.CollectorHead.CollectorIntake;
import frc.robot.commands.CollectorHead.CollectorIntakeGround;
import frc.robot.commands.CollectorHead.CollectorReverseAll;
import frc.robot.commands.CollectorHead.CollectorShoot;
import frc.robot.commands.CollectorHead.CollectorZero;
import frc.robot.commands.Elevator.ClimbDown;
import frc.robot.commands.Elevator.ClimbUp;
import frc.robot.commands.Elevator.MoveToSetpoint;
import frc.robot.commands.Elevator.MoveToSetpointShuffle;
import frc.robot.commands.Elevator.ResetElevEncoder;
import frc.robot.commands.Elevator.ResetElevator;
import frc.robot.commands.Pivot.ManualDown;
import frc.robot.commands.Pivot.ManualUp;
import frc.robot.commands.Pivot.MoveToAngle;
import frc.robot.commands.Pivot.MoveToAngleShuffle;
import frc.robot.commands.Pivot.ResetPivEncoder;
import frc.robot.commands.swervedrive.drivebase.TargetLockDriveCommandGroup;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.vision.DriveToNoteCommandGroup;
import frc.robot.commands.vision.SwapCurrentLimelight;
import frc.robot.subsystems.CollectorHeadSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
    
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final PivotSubsystem m_pivot = new PivotSubsystem();
    private static final CollectorHeadSubsystem m_collector = new CollectorHeadSubsystem();
    private static final VisionSubsystem m_limelight = new VisionSubsystem();
    
    XboxController driverXbox = new XboxController(0);
    Joystick buttonBox = new Joystick(1);
    
    private SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        configureBindings();

        addAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        
        TeleopDrive teleopDrive = new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(Constants.OperatorConstants.turnAxis),
        () -> driverXbox.getPOV(),
        () -> MathUtil.applyDeadband(driverXbox.getLeftTriggerAxis(), OperatorConstants.TRIGGER_DEADBAND) > 0,
        () -> MathUtil.applyDeadband(driverXbox.getRightTriggerAxis(), OperatorConstants.TRIGGER_DEADBAND) > 0,
        () -> driverXbox.getRawButtonPressed(XboxController.Button.kBack.value)
        );
        
        drivebase.setDefaultCommand(teleopDrive);
        
        m_collector.setDefaultCommand(new CollectorZero(m_collector));
        
        SmartDashboard.putNumber("elevSetpoint", 0);
        SmartDashboard.putNumber("pivSetpoint", 0);
        SmartDashboard.putData("MoveToSetpoint", new MoveToSetpointShuffle(m_elevator));    
        SmartDashboard.putData("MoveToAngle",new MoveToAngleShuffle(m_pivot));
        SmartDashboard.putData("ResetPivEncoder", new ResetPivEncoder(m_pivot));
        SmartDashboard.putData("ResetElevEncoder", new ResetElevEncoder(m_elevator));
    }
    
    private void configureBindings() {
        new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue((new InstantCommand(drivebase::zeroGyroWithAlliance)));
        new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
        new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value).whileTrue(new DriveToNoteCommandGroup(m_limelight, drivebase));
        new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).whileTrue(new TargetLockDriveCommandGroup(
            m_limelight,
            drivebase,        
            () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
                OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
                OperatorConstants.LEFT_X_DEADBAND),
            () -> -driverXbox.getRawAxis(Constants.OperatorConstants.turnAxis))
        );
        new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(new ManualDown(m_pivot, m_elevator));
        new JoystickButton(driverXbox, XboxController.Button.kA.value).whileTrue(new ManualUp(m_pivot, m_elevator));
        // new JoystickButton(driverXbox, XboxController.Button.kStart.value).onTrue(new InstantCommand(m_ledSubsystem::incrementDefaultMode));
        new JoystickButton(driverXbox, XboxController.Button.kRightStick.value).onTrue(new SwapCurrentLimelight(m_limelight));
        

       // new JoystickButton(driverXbox, XboxController.Button.kStart.value).onTrue(new MovePivotElev(m_pivot, m_elevator, 18, -104.5));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.zero)).onTrue(new CollectorZero(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.clear)).whileTrue(new CollectorReverseAll(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.shoot)).onTrue(new CollectorShoot(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.reset)).whileTrue(new ResetElevator(m_elevator, m_pivot)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.speaker)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevShootSpeakerFront, StateLocations.pivShootSpeakerFront));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.amp)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevShootAmp, StateLocations.pivShootAmp));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.in)).onTrue(new CollectorIntake(m_collector, m_pivot));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.rest)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevRest, StateLocations.pivRest));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.source)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevSource, StateLocations.pivSource));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.ground)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevFloor, StateLocations.pivFloor));
        
        new Trigger(() -> ButtonBox.getSliderUp(buttonBox.getRawAxis(ButtonBox.sliderAxis))).onTrue(new ClimbUp(m_elevator, m_pivot));
        new Trigger(() -> ButtonBox.getSliderDown(buttonBox.getRawAxis(ButtonBox.sliderAxis))).onTrue(new ClimbDown(m_elevator, m_pivot));
    }
    private void addAutoCommands() {
        NamedCommands.registerCommand("shoot center pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevShootSpeakerCenter, Constants.StateLocations.pivShootSpeakerCenter));
        NamedCommands.registerCommand("shoot center elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevShootSpeakerCenter));
        NamedCommands.registerCommand("shoot center pivot pos", new MoveToAngle(m_pivot,  Constants.StateLocations.pivShootSpeakerCenter));
        
        NamedCommands.registerCommand("shoot side pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevShootSpeakerSide, Constants.StateLocations.pivShootSpeakerSide));
        NamedCommands.registerCommand("shoot side elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevShootSpeakerSide));
        NamedCommands.registerCommand("shoot side pivot pos", new MoveToAngle(m_pivot,  Constants.StateLocations.pivShootSpeakerSide));

        NamedCommands.registerCommand("shoot front elev", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevShootSpeakerFront));
        NamedCommands.registerCommand("shoot front pivot", new MoveToAngle(m_pivot, Constants.StateLocations.pivShootSpeakerFront));
        
        NamedCommands.registerCommand("intake pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevFloor, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("intake pivot pos", new MoveToAngle(m_pivot, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("intake elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.pivFloor));
        
        NamedCommands.registerCommand("Shoot", new CollectorShoot(m_collector));
        NamedCommands.registerCommand("Intake", new CollectorIntakeGround(m_collector, m_pivot));

        NamedCommands.registerCommand("rest elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevRest));
        NamedCommands.registerCommand("rest piv pos", new MoveToAngle(m_pivot, Constants.StateLocations.pivRest));
    }
    
    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }
    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    public LEDStripSubsystem getLEDSubsystem() {return m_ledSubsystem;}
    public static CollectorHeadSubsystem getCollectorHeadSubsystem() {return m_collector;}
    public PivotSubsystem getPivotSubsystem() {return m_pivot;}
    public static VisionSubsystem getlimelight() {return m_limelight;}
}
