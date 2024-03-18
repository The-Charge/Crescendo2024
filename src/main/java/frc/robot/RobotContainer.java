
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.led.*;
import frc.robot.commands.swervedrive.drivebase.TargetLockDriveCommandGroup;
import frc.robot.commands.Pivot.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.vision.AutonDriveToTag;
import frc.robot.commands.vision.DriveToNoteCommandGroup;
import frc.robot.commands.vision.DriveToPoseCommandGroup;
import frc.robot.commands.vision.DriveToTagCommandGroup;
import frc.robot.commands.vision.SwapCurrentLimelight;
import frc.robot.commands.vision.UpdateRobotPose;
import frc.robot.commands.vision.swapPipeline;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.*;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.CollectorHead.*;
import frc.robot.commands.Elevator.*;
import frc.robot.Constants.StateLocations;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.Constants.ButtonBox;

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
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final LEDStripSubsystem m_ledSubsystem = new LEDStripSubsystem(); 
  private static final VisionSubsystem m_limelight = new VisionSubsystem();

 
  // CommandJoystick rotationController = new CommandJoystick(1);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick buttonBox = new Joystick(1);
  private SendableChooser<Command> autoChooser;

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private int rotationXboxAxis = 4;
  // private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final CollectorHeadSubsystem m_collector = new CollectorHeadSubsystem();
  public double elevTarget;
  public double pivTarget;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
   
    LEDVision LEDLimelight = new LEDVision(m_ledSubsystem, m_limelight);  
    // Configure the trigger bindings
    configureBindings();
    //m_collector.zero();
        autoChooser = AutoBuilder.buildAutoChooser();
        NamedCommands.registerCommand("intake pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevFloor, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("shoot pos", new MovePivotElev(m_elevator, m_pivot, Constants.StateLocations.elevShootSpeaker, Constants.StateLocations.pivShootSpeaker));
        NamedCommands.registerCommand("shoot pivot pos", new MoveToAngle(m_pivot,  Constants.StateLocations.pivShootSpeaker));
        NamedCommands.registerCommand("intake pivot pos", new MoveToAngle(m_pivot, Constants.StateLocations.pivFloor));
        NamedCommands.registerCommand("shoot elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevShootSpeaker));
        NamedCommands.registerCommand("intake elev pos", new MoveToSetpoint(m_elevator, Constants.StateLocations.elevFloor));
        NamedCommands.registerCommand("drive to note", new DriveToNoteCommandGroup(m_limelight, drivebase));
        NamedCommands.registerCommand("Shoot", new CollectorShoot(m_collector, m_pivot));
        NamedCommands.registerCommand("Intake", new CollectorIntakeGround(m_collector, m_pivot));
        NamedCommands.registerCommand("drive to center subwoofer", new AutonDriveToTag(m_limelight, drivebase, getSpeakerCenterTagID()));
        NamedCommands.registerCommand("drive to amp", new AutonDriveToTag(m_limelight, drivebase, getAmpID()));
        NamedCommands.registerCommand("drive to left subwoofer", new DriveToPoseCommandGroup(m_limelight, drivebase, new Pose2d(new Translation2d(0.66, 6.68), new Rotation2d(60))));
        NamedCommands.registerCommand("Update Robot Pose", new UpdateRobotPose(drivebase, m_limelight));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        
    
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
   * {@link CommandXboxController`
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
    

    new Trigger(() -> buttonBox.getRawButton(ButtonBox.rest)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevRest, StateLocations.pivRest));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.clear)).onTrue(new CollectorReverseAll(m_collector, m_pivot));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.zero)).onTrue(new CollectorZero(m_collector));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.shoot)).onTrue(new CollectorShoot(m_collector, m_pivot));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.shB)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevShootSpeaker, StateLocations.pivShootSpeaker));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.reset)).whileTrue(new ResetElevator(m_elevator, m_pivot));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.inS)).onTrue(new CollectorIntakeSource(m_collector));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.inG)).onTrue(new CollectorIntakeGround(m_collector, m_pivot));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.src)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevSource, StateLocations.pivSource));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.amp)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevShootAmp, StateLocations.pivShootAmp));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.gnd)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevFloor, StateLocations.pivFloor));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.elevOverride)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevClimb, StateLocations.pivClimb));
    new Trigger(() -> buttonBox.getRawButton(ButtonBox.elevOverride)).onFalse(new MoveToSetpoint(m_elevator, 5));
    //new Trigger(() -> buttonBox.getRawButton(ButtonBox.pivOverride)).onTrue(new MovePivotElev(m_elevator, m_pivot, StateLocations.elevClimb, StateLocations.pivPickupFloor));
     
    SmartDashboard.putNumber("elev setpoint", 0);
    SmartDashboard.putNumber("piv setpoint", 0);
    SmartDashboard.putData("move elev", new MoveToSetpointShuffle(m_elevator));    
    SmartDashboard.putData("move piv",new MoveToAngleShuffle(m_pivot));
    SmartDashboard.putData("reset abs encoder", new ResetPivEncoder(m_pivot));
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
  public double getSpeakerCenterTagID(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Red){
      return ApriltagConstants.RED_SPEAKER_CENTER_TAG; 
    }
    else{
      return ApriltagConstants.BLUE_SPEAKER_CENTER_TAG;
    }
  }

  public double getAmpID(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Red){
      return ApriltagConstants.RED_AMP_TAG; 
    }
    else{
      return ApriltagConstants.BLUE_AMP_TAG;
    }
  }

   public double getSpeakerSideID(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Red){
      return ApriltagConstants.RED_SPEAKER_SIDE_TAG; 
    }
    else{
      return ApriltagConstants.RED_SPEAKER_SIDE_TAG;
    }
  }

  public LEDStripSubsystem getLEDSubsystem() {return m_ledSubsystem;}
  public CollectorHeadSubsystem getCollectorHeadSubsystem() {return m_collector;}
  public static VisionSubsystem getlimelight(){return m_limelight;}
}
