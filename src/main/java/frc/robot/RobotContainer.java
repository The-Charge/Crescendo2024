
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
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
import frc.robot.commands.CollectorHead.CollectorIntakeSource;
import frc.robot.commands.CollectorHead.CollectorReverseAll;
import frc.robot.commands.CollectorHead.CollectorShoot;
import frc.robot.commands.CollectorHead.CollectorZero;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Pivot.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import frc.robot.subsystems.*;
import frc.robot.commands.PivotElevator;
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
  // private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  //     "swerve"));
  private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // private final PivotSubsystem m_pivot = new PivotSubsystem();


  private final CollectorHeadSubsystem m_collector = new CollectorHeadSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick buttonBox = new Joystick(1);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private int rotationXboxAxis = 4;
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //m_collector.zero();

    // pivotElevator pivotElevator = new pivotElevator(
    //   m_elevator, m_pivot,
    //   () -> {
    //     if(buttonBox.getRawButtonPressed(1)) return 0;
    //     else if(buttonBox.getRawButtonPressed(2)) return 1;
    //     else if(buttonBox.getRawButtonPressed(3)) return 2;
    //     else if(buttonBox.getRawButtonPressed(4)) return 3;
    //     else if(buttonBox.getRawButtonPressed(5)) return 4;
    //     else if(buttonBox.getRawButtonPressed(6)) return 5;
    //     else if(buttonBox.getRawButtonPressed(7)) return 6;
    //     else if(buttonBox.getRawButtonPressed(8)) return 7;

    //     return -1;
    //   }
    // );

    // TeleopDrive teleopDrive = new TeleopDrive(
    //   drivebase,
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> -driverXbox.getRawAxis(rotationXboxAxis)
    // );

    // drivebase.setDefaultCommand(teleopDrive);

    CollectorZero collectorZero = new CollectorZero(m_collector);
    m_collector.setDefaultCommand(collectorZero);

    m_elevator.setDefaultCommand(new MoveElevWithJoystick(m_elevator, () -> buttonBox.getY()));
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
    
    //new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    return null;
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }

  public LEDStripSubsystem getLEDSubsystem() {return m_ledSubsystem;}
  // public CollectorHeadSubsystem getCollectorHeadSubsystem() {return m_collector;}
}
