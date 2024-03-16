package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class MoveToAngle extends Command {
    
    public double angleGoal;
    private final PivotSubsystem m_pivot;
    
    public MoveToAngle(PivotSubsystem subsystem, double angle) {
        m_pivot = subsystem;
        addRequirements(subsystem);
        
        angleGoal = angle;
    }
    
    @Override
    public void initialize() {
        m_pivot.pivotToAngle(angleGoal);
        //SmartDashboard.putBoolean("auto", true);
    }
    @Override
    public boolean isFinished() {
        // return true;
        return m_pivot.isAtTarget();
    }
    
    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putBoolean("auto", false);
    }
}
