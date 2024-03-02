package frc.robot.commands.CollectorHead;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorHeadSubsystem;

public class CollectorShoot extends Command {

    private CollectorHeadSubsystem head;
    Timer timeout, feedTimer;
    boolean hasStartedIndexers, hasSetTime;

    public CollectorShoot(CollectorHeadSubsystem head) {
        this.head = head;
    
    }

    @Override
    public void initialize() {
        timeout = new Timer();
        feedTimer = null;
        timeout.start();

        head.resetTargetCounter();
        head.spinShooter(CollectorHeadSubsystem.Direction.FORWARD, 1);
        hasStartedIndexers = false;
        hasSetTime = false;
    }

    @Override
    public void execute() {
      
        if(head.shootIsATarget(10000) && !hasStartedIndexers) {
            head.spinIndexer(CollectorHeadSubsystem.Direction.FORWARD, 1);
            head.spinIndexer(CollectorHeadSubsystem.Direction.FORWARD, 1);
            hasStartedIndexers = true;
        }

        if(!head.getNoteSensor1() && !head.getNoteSensor2() && !hasSetTime) {
            feedTimer = new Timer();
            feedTimer.start();
            hasSetTime = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        head.zero();
    }

    @Override
    public boolean isFinished() {
        if(hasStartedIndexers) {
            SmartDashboard.putNumber("feed timer", feedTimer.get());
        }
        return timeout.hasElapsed(12) || (feedTimer == null ? false : feedTimer.hasElapsed(1.5));
    }
}
