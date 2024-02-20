package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
    public double x_error, y_error;
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

        /*Update note pose
        x_error = Math.pow((1.82 * ta),(-0.468));
        SmartDashboard.putNumber("x_Error", x_error);
        notepose = new Pose2d(new Translation2d(robotpose.getX() + x_error ,robotpose.getY() + y_error), robotpose.getRotation());
        */
        //SmartDashboard stuff
        
        SmartDashboard.putNumber("Robot Orientation", robotpose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robotx", robotpose.getX());
        SmartDashboard.putNumber("Roboty", robotpose.getY());

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

    public void swapPipeline(){
        if (getpipe == 0.0){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1.0);
        }
        else{
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0.0);
        }
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


