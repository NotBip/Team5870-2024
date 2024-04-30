

 package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonPipelineResult;
 
 public class Vision {
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private double lastEstTimestamp = 0;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;

     private Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
     public static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
     public static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
     
     public Vision() {
         camera = new PhotonCamera("limelight");
 
         photonEstimator =
                 new PhotonPoseEstimator(
                    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField() , PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.
             cameraSim = new PhotonCameraSim(camera, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
 
             cameraSim.enableDrawWireframe(true);
         }
     }
 
     public PhotonPipelineResult getLatestResult() {
         return camera.getLatestResult();
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         var visionEst = photonEstimator.update();
         double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
         if (Robot.isSimulation()) {
             visionEst.ifPresentOrElse(
                     est ->
                             getSimDebugField()
                                     .getObject("VisionEstimation")
                                     .setPose(est.estimatedPose.toPose2d()),
                     () -> {
                         if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                     });
         }
         if (newResult) lastEstTimestamp = latestTimestamp;
         return visionEst;
     }
 
     /**
      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
      * This should only be used when there are targets visible.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      */
     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
         var estStdDevs = kSingleTagStdDevs;
         var targets = getLatestResult().getTargets();
         int numTags = 0;
         double avgDist = 0;
         for (var tgt : targets) {
             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
             if (tagPose.isEmpty()) continue;
             numTags++;
             avgDist +=
                     tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
         }
         if (numTags == 0) return estStdDevs;
         avgDist /= numTags;
         // Decrease std devs if multiple targets are visible
         if (numTags > 1) estStdDevs = kMultiTagStdDevs;
         // Increase std devs based on (average) distance
         if (numTags == 1 && avgDist > 4)
             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
 
         return estStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }
 }