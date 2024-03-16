package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class ResetPivEncoder extends Command {
    
    private PivotSubsystem piv;
    
    public ResetPivEncoder(PivotSubsystem piv) {
        this.piv = piv;
    }

    @Override
    public void initialize() {
        piv.resetEncoder();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
