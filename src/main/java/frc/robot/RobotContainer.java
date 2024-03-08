// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.led.*;
import frc.robot.commands.swervedrive.drivebase.TargetLockDriveCommandGroup;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.commands.vision.DriveToNoteCommandGroup;
import frc.robot.commands.vision.DriveToTagCommandGroup;
import frc.robot.commands.vision.SetCenterSpeakerPriorityID;
import frc.robot.commands.vision.SwapCurrentLimelight;
import frc.robot.commands.vision.UpdateCameraPose;
import frc.robot.commands.vision.UpdateRobotPose;
import frc.robot.commands.vision.swapPipeline;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem(); 
  private static final VisionSubsystem m_limelight = new VisionSubsystem();

 
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private int rotationXboxAxis = 4;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    UpdateRobotPose updateRobotPose = new UpdateRobotPose(drivebase, m_limelight);
    LEDVision LEDLimelight = new LEDVision(m_ledSubsystem, m_limelight);
  
    NamedCommands.registerCommand("Update Robot Pose", updateRobotPose);
    // Configure the trigger bindings
    configureBindings();


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
    m_ledSubsystem.setDefaultCommand(LEDLimelight);
 
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
    new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(new RepeatCommand(new UpdateRobotPose(drivebase, m_limelight)));
    
    new JoystickButton(driverXbox, XboxController.Button.kStart.value).whileTrue(new DriveToTagCommandGroup(m_limelight, drivebase));
    new JoystickButton(driverXbox, XboxController.Button.kA.value).whileTrue(new DriveToNoteCommandGroup(m_limelight, drivebase));
    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).whileTrue(new TargetLockDriveCommandGroup(
        m_limelight,
        drivebase,        
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(rotationXboxAxis))
            );

    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value).onTrue(new swapPipeline(m_limelight));
    new JoystickButton(driverXbox, XboxController.Button.kLeftStick.value).onTrue(new SwapCurrentLimelight(m_limelight));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Path");
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public LEDStripSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }
  public static VisionSubsystem getlimelight(){
    return m_limelight;
  }
  /*public static VisionSubsystem getlimelightShooter(){
    return m_limelightshooter;
  }
   * 
   */


}