package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonLL extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("limelight");  

  private double yaw;

  private double pitch;

  private double area;

  private double Id;

  private double xMeters;

  private double yMeters;

  private boolean hasTargets;

  private Transform3d transform3d;
  


      
    
  /** Creates a new ExampleSubsystem. */
  public PhotonLL() {
   

    
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets){
      var target = result.getBestTarget();
  
      // GET DATA:
      yaw = Math.toDegrees(target.getBestCameraToTarget().getRotation().getZ());
      pitch = target.getPitch();
      area = target.getArea();
      Id = target.getFiducialId();
      xMeters = target.getBestCameraToTarget().getTranslation().getX(); 
      yMeters = target.getBestCameraToTarget().getTranslation().getY(); 
      transform3d = result.getMultiTagResult().estimatedPose.best; 


      
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("Apriltag Id", Id);
      SmartDashboard.putNumber("X-METERS", xMeters);
      SmartDashboard.putNumber("Y-METERS", yMeters);
      SmartDashboard.putString("CAM TO TARGET", target.getBestCameraToTarget().toString()); 
      
    } 

  }

    private static PhotonLL instance;
    public static  PhotonLL getInstance(){
      if (instance == null){
        instance = new PhotonLL();
      }
      
      return instance;
    }
    
    public boolean hasValueTargets(){
      return hasTargets;
    }

    public double getYaw(){
      return yaw;
    }

    public double getYDistance(){
      return yMeters;
    }

    public double getXDistance(){
      return xMeters;
    }

    public double getArea(){
      return area;
    }

    public double getFiducialId() { 
      return Id; 
    }

    public Transform3d geTransform3d() { 
      return transform3d;
    }
}

