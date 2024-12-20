// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CollectorHead.CollectorZero;
import frc.robot.commands.led.*;

import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

public class Robot extends TimedRobot {
    
    private static Robot instance;
    public RobotContainer m_container;
    
    private Command m_autoCommand;
    private Timer disabledTimer;
    
    public Robot() {
        instance = this;
    }
    public static Robot getInstance() {
        return instance;
    }
    public static RobotContainer getContainer() {
        return instance.m_container;
    }

    @Override
    public void robotInit() {
        m_container = new RobotContainer();
        
        disabledTimer = new Timer();
        
        // Connect to 172.22.11.2:2011 to see fixed limelight
         PortForwarder.add(2011, "limelight-fixed.local", 5800);
         PortForwarder.add(2011, "limelight-fixed.local", 5801);
         PortForwarder.add(2011, "limelight-fixed.local", 5805);

        // Connect to 172.22.11.2:2012 to see shooter limelight
         PortForwarder.add(2012, "limelight-shooter.local", 5800);
         PortForwarder.add(2012, "limelight-shooter.local", 5801);
         PortForwarder.add(2012, "limelight-shooter.local", 5805);
    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledInit() {
        m_container.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
        
        // new DisableLEDs(m_robotContainer.getLEDSubsystem()).schedule();
        new CollectorZero(RobotContainer.getCollectorHeadSubsystem()).schedule();

        new LEDChase(m_container.getLEDSubsystem(), Color.kLime, Color.kGold, () -> false).schedule();

    }
    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            m_container.setMotorBrake(false);
            disabledTimer.stop();
        }
    }
    
    @Override
    public void autonomousInit() {
        // new CollectorZero(m_robotContainer.getCollectorHeadSubsystem()).schedule();
        
        m_container.setMotorBrake(true);
        m_autoCommand = m_container.getAutonomousCommand();
        
        if (m_autoCommand != null) {
            m_autoCommand.schedule();
        }
        new SetLEDBrightness(m_container.getLEDSubsystem(), 0.5, false);
        new LEDVision(m_container.getLEDSubsystem()).schedule();
    }
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
        // new CollectorZero(m_robotContainer.getCollectorHeadSubsystem()).schedule();
        
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
        m_container.setDriveMode();
        m_container.setMotorBrake(true);
        new SetLEDBrightness(m_container.getLEDSubsystem(), 0.5, false);
        new LEDVision(m_container.getLEDSubsystem()).schedule();
    }
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        try {
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        catch (IOException e) {throw new RuntimeException(e);}
    }
    @Override
    public void testPeriodic() {}
    
    @Override
    public void simulationInit() {}
    @Override
    public void simulationPeriodic() {}
}
