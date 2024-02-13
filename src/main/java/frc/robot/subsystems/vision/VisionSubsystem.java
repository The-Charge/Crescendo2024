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
    public double tx;               //X-offset
    public double ty;               //Y-offset
    public double tv;               //Target Identification
    public double ta;               //Area of tag
    public double tid;              //Tag id
    public double tl;               //latency contribution
    public double cl;               //Capture pipeline latency
    public double getpipe;                 //get current pipeline
    public double limelightlatency; //tl + cl
    public double thor;
    public Pose2d robotpose;        //Robot in Fieldspace (blue side)

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
        tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
        tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
        cl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl").getDouble(0);
        getpipe = NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0);
        thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
        //Read pose-specific values
        robotpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        
        
        //Update rest of vars with simple logic
        updateTargetIdentified();
        limelightlatency = tl + cl;

        //SmartDashboard stuff
        SmartDashboard.putNumber("LimeLight TV", tv);
        SmartDashboard.putNumber("LimeLight TX", tx);
        SmartDashboard.putNumber("LimeLight TY", ty);
        SmartDashboard.putNumber("LimeLight TA", ta);
        SmartDashboard.putNumber("Tag Id", tid);
        SmartDashboard.putBoolean("Target Identified?", targetIdentified);
        SmartDashboard.putNumber("Robot Orientation", robotpose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robotx", robotpose.getX());
        SmartDashboard.putNumber("Roboty", robotpose.getY());
        SmartDashboard.putNumber("Current pipeline", getpipe);
        SmartDashboard.putNumber("horizontal countour length", thor);
    }
    
    //convert tv value to boolean
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


    //Getters
    public double gettx(){
        return tx;
    }
    public double getty(){
        return ty;
    }
    public double gettv(){
        return tv;
    }
    public double getta(){
        return ta;
    }
    public double gettid(){
        return tid;
    }
    public double getthor(){
        return thor;
    }
    public double getcurrentpipeline(){
        return getpipe;
    }
    public double getLimelightLatency(){
        return limelightlatency;
    }
    public Pose2d getRobotFieldPose(){
        return robotpose;
    }
}


