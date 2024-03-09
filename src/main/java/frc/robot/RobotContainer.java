
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
import frc.robot.commands.CollectorHead.CollectorIntakeGround;
import frc.robot.commands.CollectorHead.CollectorIntakeSource;
import frc.robot.commands.CollectorHead.CollectorReverseAll;
import frc.robot.commands.CollectorHead.CollectorShoot;
import frc.robot.commands.CollectorHead.CollectorZero;
import frc.robot.commands.Pivot.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.*;
import frc.robot.commands.MovePivotElev;
import frc.robot.commands.PivotElevator;
import frc.robot.commands.StateMachine;
import frc.robot.commands.Climber.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Pivot.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // private final PivotSubsystem m_pivot = new PivotSubsystem();


  private final CollectorHeadSubsystem m_collector = new CollectorHeadSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick buttonBox = new Joystick(1);
  private SendableChooser<Command> autoChooser;

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private int rotationXboxAxis = 4;
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  public double elevTarget;
  public double pivTarget;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //m_collector.zero();
        autoChooser = AutoBuilder.buildAutoChooser();
        NamedCommands.registerCommand("intake pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevPickupFloor, Constants.StateLocations.pivPickupFloor));
        NamedCommands.registerCommand("shoot pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevHighRear, Constants.StateLocations.pivHighRear));
        NamedCommands.registerCommand("Shoot", new CollectorShoot(m_collector));
        NamedCommands.registerCommand("Intake", new CollectorIntakeGround(m_collector, m_pivot));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        /*   
      } */
    
      // in robot in auton init: m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    TeleopDrive teleopDrive = new TeleopDrive(drivebase,
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRawAxis(rotationXboxAxis),
      () -> driverXbox.getPOV(),
      () -> MathUtil.applyDeadband(driverXbox.getLeftTriggerAxis(), OperatorConstants.TRIGGER_DEADBAND) > 0,
      () -> MathUtil.applyDeadband(driverXbox.getRightTriggerAxis(), OperatorConstants.TRIGGER_DEADBAND) > 0,
      () -> driverXbox.getRawButtonPressed(XboxController.Button.kBack.value)
    );

    drivebase.setDefaultCommand(teleopDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    new Trigger(() -> buttonBox.getRawButton(3)).onTrue(new CollectorIntakeSource(m_collector));
    new Trigger(() -> buttonBox.getRawButton(4)).onTrue(new CollectorReverseAll(m_collector));
    new Trigger(() -> buttonBox.getRawButton(2)).onTrue(new CollectorShoot(m_collector));
    new Trigger(() -> buttonBox.getRawButton(1)).onTrue(new CollectorZero(m_collector));
    new Trigger(() -> buttonBox.getRawButton(5)).onTrue(new MoveToSetpoint(m_elevator, 5));
    new Trigger(() -> buttonBox.getRawButton(6)).onTrue(new MoveToSetpoint(m_elevator, 19));
    new Trigger(() -> buttonBox.getRawButton(7)).onTrue(new MoveToSetpoint(m_elevator, 27));
    new Trigger(() -> buttonBox.getRawButton(8)).onTrue(new MoveToSetpoint(m_elevator, 0));

    
    new Trigger(() -> buttonBox.getRawButton(9)).onTrue(new MoveToAngle(m_pivot, -28.52));

    new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    elevTarget = SmartDashboard.getNumber(" elev setpoint",  0.0);
    SmartDashboard.putData("move elev", new MoveToSetpoint(m_elevator, elevTarget));
    pivTarget = SmartDashboard.getNumber("piv setpoint", 0.0);
    SmartDashboard.putData("move piv",new MoveToAngle(m_pivot, pivTarget));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


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
}
