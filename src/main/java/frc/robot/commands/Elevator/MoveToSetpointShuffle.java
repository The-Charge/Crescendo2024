package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToSetpointShuffle extends Command {
    
    private final ElevatorSubsystem elevatorSub;
    
    public MoveToSetpointShuffle(ElevatorSubsystem elev) {
        elevatorSub = elev;
        addRequirements(elev);
    }
    
    @Override
    public void initialize() {
        elevatorSub.goToPosition(SmartDashboard.getNumber("elevSetpoint", 0));
    }
    @Override
    public boolean isFinished() {
        return elevatorSub.isAtTarget();
        // return true;
    }
}
