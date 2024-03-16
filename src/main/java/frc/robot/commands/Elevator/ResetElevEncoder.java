package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ResetElevEncoder extends Command {
    
    private ElevatorSubsystem elev;

    public ResetElevEncoder(ElevatorSubsystem elev) {
        this.elev = elev;
        addRequirements(this.elev);
    }

    @Override
    public void initialize() {
        elev.setAsZero();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
