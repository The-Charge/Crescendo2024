// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.commands.led.*;
import frc.robot.commands.Pivot.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.CollectorHead.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Pivot.*;
import frc.robot.Constants.StateLocations;
import frc.robot.Constants.ButtonBox;

public class RobotContainer {
    
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final PivotSubsystem m_pivot = new PivotSubsystem();
    private final CollectorHeadSubsystem m_collector = new CollectorHeadSubsystem();
    
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
        new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue((new InstantCommand(drivebase::zeroGyro)));
        new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
        
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.rest)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevRest, StateLocations.pivRest));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.clear)).whileTrue(new CollectorReverseAll(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.zero)).onTrue(new CollectorZero(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.shoot)).onTrue(new CollectorShoot(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.shB)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevShootSpeaker, StateLocations.pivShootSpeaker));
        // new Trigger(() -> buttonBox.getRawButton(ButtonBox.reset)).whileTrue(new ResetElevator(m_elevator, m_pivot));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.inS)).onTrue(new CollectorIntakeSource(m_collector));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.inG)).onTrue(new CollectorIntakeGround(m_collector, m_pivot));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.src)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevSource, StateLocations.pivSource));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.amp)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevShootAmp, StateLocations.pivShootAmp));
        new Trigger(() -> buttonBox.getRawButton(ButtonBox.gnd)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevFloor, StateLocations.pivFloor));
        // new Trigger(() -> buttonBox.getRawButton(ButtonBox.elevOverride)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevClimb, StateLocations.pivClimb));
        // new Trigger(() -> buttonBox.getRawButton(ButtonBox.elevOverride)).onFalse(new MoveToSetpoint(m_elevator, 5));
        //new Trigger(() -> buttonBox.getRawButton(ButtonBox.pivOverride)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevClimb, StateLocations.pivPickupFloor));

        new Trigger(() -> buttonBox.getRawAxis(0) < -0.5).onTrue(new ClimbUp(m_elevator, m_pivot));
        new Trigger(() -> buttonBox.getRawAxis(0) > 0.5).onTrue(new ClimbDown(m_elevator, m_pivot));
    }
    private void addAutoCommands() {
        NamedCommands.registerCommand("intake pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevFloor, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("shoot pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevShootSpeaker, Constants.StateLocations.pivShootSpeaker));
        NamedCommands.registerCommand("shoot pivot pos", new MoveToAngle(m_pivot,  Constants.StateLocations.pivShootSpeaker));
        NamedCommands.registerCommand("intake pivot pos", new MoveToAngle(m_pivot, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("shoot elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevShootSpeaker));
        NamedCommands.registerCommand("intake elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("Shoot", new CollectorShoot(m_collector));
        NamedCommands.registerCommand("Intake", new CollectorIntakeGround(m_collector, m_pivot));

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
    public CollectorHeadSubsystem getCollectorHeadSubsystem() {return m_collector;}
    public PivotSubsystem getPivotSubsystem() {return m_pivot;}
}
