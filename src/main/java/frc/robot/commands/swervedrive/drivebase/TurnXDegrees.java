package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurnXDegrees extends Command {

    SwerveSubsystem m_swerve;
    Double m_degrees;

    public TurnXDegrees(SwerveSubsystem swerve, double degrees) {
        m_swerve = swerve;
        m_degrees = degrees;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        Pose2d pose = m_swerve.getPose();
        Pose2d pose2 = new Pose2d(new Translation2d(7,9), pose.getRotation().rotateBy(new Rotation2d(m_degrees)));
        m_swerve.driveToPose(pose2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
