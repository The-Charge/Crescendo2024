// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.led.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

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
  private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem();
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private int rotationXboxAxis = 4;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    if (RobotBase.isSimulation()) {
      rotationXboxAxis = 2;
    }

    TeleopDrive teleopDrive = new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(rotationXboxAxis));

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, XboxController.Button.kA.value).whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    new JoystickButton(driverXbox, XboxController.Button.kX.value)
        .whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    driverController.button(1).onTrue(new SetLEDBrightness(m_ledSubsystem, 0.25));
    driverController.button(2).onTrue(new SetLEDBrightness(m_ledSubsystem, 1));
    
    driverController.button(3).onTrue(new DisableLEDs(m_ledSubsystem));

    driverController.button(4).onTrue(new LEDRainbow(m_ledSubsystem, () -> driverController.getHID().getRawButton(5))); //end when button 5 is pressed
    
    driverController.button(7).onTrue(new SetLEDRange(m_ledSubsystem, 0, 23, new Color("#FF0000")));
    driverController.button(8).onTrue(new SetLEDRange(m_ledSubsystem, 0, 23, new Color("#00FF00")));
    driverController.button(9).onTrue(new SetLEDRange(m_ledSubsystem, 23, 45, new Color("#0000FF")));
    driverController.button(10).onTrue(new SetLEDRange(m_ledSubsystem, 23, 45, new Color("#FFFF00")));
    driverController.button(11).onTrue(new SetLEDRange(m_ledSubsystem, 0, 45, new Color("#00FFFF")));
    driverController.button(12).onTrue(new SetLEDRange(m_ledSubsystem, 0, 45, new Color("#FF00FF")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Path", true);
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public LEDStripSubsystem getLEDSubsystem() {return m_ledSubsystem;}
}
