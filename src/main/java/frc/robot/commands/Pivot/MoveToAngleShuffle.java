package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class MoveToAngleShuffle extends Command {
    
    private final PivotSubsystem m_pivot;
    
    public MoveToAngleShuffle(PivotSubsystem subsystem) {
        m_pivot = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        m_pivot.pivotToAngle(SmartDashboard.getNumber("pivSetpoint", 0));
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
