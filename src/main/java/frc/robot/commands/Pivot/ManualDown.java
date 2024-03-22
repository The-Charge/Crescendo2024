package frc.robot.commands.Pivot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ManualDown extends Command{
    private final PivotSubsystem m_pivot;
    private final ElevatorSubsystem m_elevator;
    private SlewRateLimiter movePiv;
    public ManualDown(PivotSubsystem subsystem, ElevatorSubsystem elev) {
        m_pivot = subsystem;
        addRequirements(subsystem);
        m_elevator = elev;
        movePiv = new SlewRateLimiter(5);
    }
    @Override
    public void initialize() {
        movePiv.reset(m_pivot.getAngle());
    }

    @Override
    public void execute() {
        if(Math.abs(m_elevator.getPosition() - Constants.StateLocations.elevShootSpeakerCenter) <= Constants.Elevator.rangeSize) {
            double newpos = movePiv.calculate(Constants.StateLocations.pivShootSpeakerCenter - 6.0);
            m_pivot.setManualPivotOverride(newpos);
            m_pivot.pivotToAngle(newpos);
        }
       
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {

    }

    
}
