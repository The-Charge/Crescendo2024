package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.commands.leds.LEDAprilTag;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase{
  
    public boolean targetIdentified = false;
    public double tx;
    public double ty;
    public double tv;
    public double ta;
    public Pose2d robotpose;
    public Pose3d targetCamPose;
    public VisionSubsystem(){
        
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      updateLimelightTracking();
    }
  
    //updates limelight tracked values and puts on SmartDashboard
    public void updateLimelightTracking(){
        //Read basic values 
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        
        //Read pose-specific values
        robotpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        targetCamPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
        

         updateTargetIdentified();
        //SmartDashboard stuff
        SmartDashboard.putNumber("LimeLight TV", tv);
        SmartDashboard.putNumber("LimeLight TX", tx);
        SmartDashboard.putNumber("LimeLight TY", ty);
        SmartDashboard.putNumber("LimeLight TA", ta);
        SmartDashboard.putBoolean("Target Identified?", targetIdentified);
        SmartDashboard.putNumber("Robot Orientation", robotpose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robotx", robotpose.getX());
        SmartDashboard.putNumber("Roboty", robotpose.getY());
    }
    
    //convert tv value to boolean
    //added led on when detected for tracking purposes.
    public boolean updateTargetIdentified(){
        if (tv > 0) {
            targetIdentified = true;
        }
        else {
            targetIdentified = false;

        }
        return targetIdentified;
    }
 
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public double gettx(){
        return tx;
    }
    public double gettv(){
        return tv;
    }
    public Pose2d getRobotFieldPose(){
        return robotpose;
    }
    public Pose3d gettargetCamPose(){
        return targetCamPose;
    }
    public double getTargetRobotx(){
        return targetCamPose.getX();
    }
    public double getTargetRoboty(){
        return targetCamPose.getY();
    }
}


