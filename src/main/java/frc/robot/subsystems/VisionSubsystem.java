package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FixedLLConstants;
import frc.robot.Constants.ShooterLLConstants;

public class VisionSubsystem extends SubsystemBase{
    public String limelightname = ShooterLLConstants.SHOOTER_LL_NAME;   //first camera initialiation
    public boolean targetIdentified = false;    
    public double tx;               //X-offset
    public double ty;               //Y-offset
    public double tv;               //Target Identification
    public double ta;               //Area of tag
    public double tid;              //Tag id
    public double tl;               //latency contribution
    public double cl;               //Capture pipeline latency
    public double getpipe;          //get current pipeline
    public double limelightlatency; //tl + cl
    public double distance;         //distance to target
    public Pose2d robotpose;        //Robot in Fieldspace (blue side)
    public double[] campose = new double[6];          //3D tranform of thecamerai n the coordinate system of the robot
    public double prevtag;

    public VisionSubsystem(){
    
    }

    @Override
    public void periodic(){
      // This method will be called once per scheduler run
      updateLimelightTracking();
    }
  
    //updates limelight tracked values and puts on SmartDashboard
    public void updateLimelightTracking(){ 
        //Read general target values
        tv = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("ta").getDouble(0);
        tid = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tid").getDouble(0);
        tl = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("tl").getDouble(0);
        cl = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("cl").getDouble(0);
        getpipe = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("getpipe").getDouble(0);

        //Read pose-specific values
        campose = NetworkTableInstance.getDefault().getTable(limelightname).getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
        robotpose = LimelightHelpers.getBotPose2d_wpiBlue(limelightname);

        SmartDashboard.putString("limelightname: ", limelightname);
        //Update rest of variables
        limelightlatency = (tl + cl) / 1000.0;                 //ms
        distance = Math.pow((ta * 1.82), -0.468);   //formula: 1.82x^-0.468, in meters

        SmartDashboard.putNumber("ll latency", limelightlatency);
        if (tv > 0){
            LimelightHelpers.setLEDMode_ForceBlink(limelightname);
        }
        else{
            LimelightHelpers.setLEDMode_ForceOn(limelightname);
        }
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void swapPipeline(){
        if (limelightname == FixedLLConstants.FIXED_LL_NAME){
            if (getpipe != FixedLLConstants.FIXED_APRILTAG_PIPELINE){
                NetworkTableInstance.getDefault().getTable(limelightname).getEntry("pipeline").setNumber(FixedLLConstants.FIXED_APRILTAG_PIPELINE); 
            }
            else{
                NetworkTableInstance.getDefault().getTable(limelightname).getEntry("pipeline").setNumber(FixedLLConstants.FIXED_DRIVER_PIPELINE);
            }
        }
        else{
            if (getpipe != ShooterLLConstants.SHOOTER_NEURAL_NETWORK_PIPELINE){
                NetworkTableInstance.getDefault().getTable(limelightname).getEntry("pipeline").setNumber(ShooterLLConstants.SHOOTER_NEURAL_NETWORK_PIPELINE); 
            }   
            else{
                NetworkTableInstance.getDefault().getTable(limelightname).getEntry("pipeline").setNumber(ShooterLLConstants.SHOOTER_DRIVER_PIPELINE);
            }
        }
       
    }

    public void swapCurrentLimelightName(){
        LimelightHelpers.setLEDMode_ForceOff(limelightname);
        if (limelightname == FixedLLConstants.FIXED_LL_NAME){
            limelightname = ShooterLLConstants.SHOOTER_LL_NAME;
        }
        else{
            limelightname = FixedLLConstants.FIXED_LL_NAME;
        }
        LimelightHelpers.setLEDMode_ForceOn(limelightname);
    }


    //Setters
    public void setPrevTag(double tagid){
        prevtag = tagid;
    }
    public void setCurrentLimelightName(String newname){
        LimelightHelpers.setLEDMode_ForceOff(limelightname);
        limelightname = newname;
        LimelightHelpers.setLEDMode_ForceOn(limelightname);
    }
    
    public void setPipeline(double index){
        NetworkTableInstance.getDefault().getTable(limelightname).getEntry("pipeline").setNumber(index);
    }
    
    
    public void setCameraPose(){ //elevator and pivot will change camera location, must adjust to that
        if (limelightname == ShooterLLConstants.SHOOTER_LL_NAME){
            double x = 0;
            double z = 0;
            double pitch = 0; //other 3 values will not change hopefully
        campose[0] += x;    //change x
        campose[2] += z;    //change z
        campose[4] += pitch;    //change pitch
        NetworkTableInstance.getDefault().getTable(limelightname).getEntry("camerapose_robotspace_set").setDoubleArray(campose);    //change later
        }
        
    }

    public void setPriorityID(double id){
         NetworkTableInstance.getDefault().getTable(limelightname).getEntry("priorityid").setNumber(id);   
    }
    //Getters
    public String getlimelightshootername(){
        return limelightname;
    }
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
    public double getdistance(){
        return distance;
    }
    public double getcurrentpipeline(){
        return getpipe;
    }
    public double getlimelightshooterLatency(){
        return limelightlatency;
    }
    public double getprevtag(){
        return prevtag;
    }
    public Pose2d getRobotFieldPose(){
        return robotpose;
    }
    public double[] getCampose(){
        return campose;
    }    
    

}
