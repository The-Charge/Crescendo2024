
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import frc.robot.subsystems.*;

import frc.robot.commands.Climber.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Pivot.*;
import frc.robot.commands.Shooter.*;

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
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private int rotationXboxAxis = 4;

  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    if (RobotBase.isSimulation()) {
      rotationXboxAxis = 2;
    }

    TeleopDrive teleopDrive = new TeleopDrive(
      drivebase, m_elevator, m_pivot, 
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRawAxis(rotationXboxAxis),
      () -> false,
      () -> false,
      () -> false,
      () -> false,
      () -> false,
      () -> false,
      () -> false,
      () -> false
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
    driverController.button(1).onTrue(new MoveElevatorToSetpoint(m_elevator, 0));
    driverController.button(2).onTrue(new MoveElevatorToSetpoint(m_elevator, 500));

    // new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue((new
    // InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(new
    // RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }
}
