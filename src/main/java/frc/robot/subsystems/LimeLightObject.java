package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightObject extends SubsystemBase{

    private static LimeLightObject instance; //Instance for the limelight

    private boolean objectIsSeen = false; //Boolean for if the apriltag has been seen

    private double x; //April tag x offset

    private double y; //April tag offset

    private double a; //April tag target area

    private double v; //Whether the limelight has any valid targets (0 or 1)

    private double s; //April Tag skew

    private double yaw; //April tag yaw

    private int pipelineNumber; //the number of the pipeline in use




    @Override
    public void periodic(){

        /**Get the limelight NetworkTable
         */
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        /** Get the entry por the pipeline */
        table.getEntry("pipeline").setNumber(pipelineNumber);
        
        /**Get array of robot pose entries */
        double[] robotPose = table.getEntry("botpose").getDoubleArray(new double[6]);

        /**Get Limelight tx */
        NetworkTableEntry tx = table.getEntry("tx");
        x = -tx.getDouble(0.0);
    
        /**Get Limelight ty */
        NetworkTableEntry ty = table.getEntry("ty");
        y = ty.getDouble(0.0);
    
        /**Get Limelight ta */
        NetworkTableEntry ta = table.getEntry("ta");
        a = ta.getDouble(0.0);
    
        /**Get Limelight tv */
        NetworkTableEntry tv = table.getEntry("tv");
        v = tv.getDouble(0.0);

        /**Get Limelight ts */
        NetworkTableEntry ts = table.getEntry("ts");
        s = ts.getDouble(0.0);
    
    
        /**Get limelight yaw */
        yaw = -robotPose[5];

        /**Convert the Limelight v parameter from int to boolean*/
        if(v > 0){
            objectIsSeen = true;
          } else{
            objectIsSeen = false;
          } 

        /**SHUFFLEBOARD */
        
        SmartDashboard.putNumber("Yaw", yaw);

        SmartDashboard.putNumber("X", x);
        
        SmartDashboard.putNumber("S", s);
        
        SmartDashboard.putNumber("Y", y);
        
        SmartDashboard.putNumber("A", a);
        
        SmartDashboard.putBoolean("isthiswhatithinkitis?", objectIsSeen);
        
    }
    
    /**
     * Void to set the number of the pipeline to use
     * @param alingToAprilTag if it will align to april tag
     */
    public void alingToAprilTag(boolean alingToAprilTag){
        if(!alingToAprilTag){
            pipelineNumber = 1;      
        } else {
            pipelineNumber = 0;
        }
    }

    /**
     * Get the offsets for the limelight depending on the target to search
     * @param alingToAprilTag if it will align to april tag
     * @return a {@limelightOffsets}
     */
   /*  public limelightOffsets getOffsets(boolean alingToAprilTag){

        if(alingToAprilTag){
            offsets = aprilTag.offsets;
        } else if(!alingToAprilTag){
            offsets = reflectiveTape.offsets;
        }

        return offsets;
    }

    /**@return Apriltag xOffset */
    public double getXLimelight(){        
        return x;
    }

    /**@return Apriltag yOffset */
    public double getYLimelight(){
        return y;
    }

    /**@return Apriltag Target Area */
    public double getALimelight(){
        return a;
    }

    /**@return Whether the limelight has any valid targets */
    public double getVLimelight(){
        return v;
    }

    /**@return Apriltag Yaw */
    public double getYaw(){
        return yaw;
    }

    /**@return Whether the limelight has any valid targets in Boolean */
    public boolean getObjectIsSeen(){
        return objectIsSeen;
    }

    /**
     * The instance of the limelight
     * @return The instance of the limelight
     */
    public static LimeLightObject getInstance(){
        if(instance == null){
            instance = new LimeLightObject();
        }
        return instance;
    }
   
}
