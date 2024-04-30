// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSim extends SubsystemBase {
  private SimSwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Field2d field = new Field2d();
  private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
  private Vision vision = new Vision(); 
  public SimGyro gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  
  public SwerveSim() {
    gyro = new SimGyro();
    modules = new SimSwerveModule[]{
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule()
    };

    kinematics = DriveConstants.kDriveKinematics; 
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      AutoConstants.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );
    
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);
    poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getRotation2d(), getPositions(), new Pose2d(), stateStdDevs, visionStdDevs); 
    
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project

    gyro.updateRotation(getSpeeds().omegaRadiansPerSecond); 
    odometry.update(gyro.getRotation2d(), getPositions());
    poseEstimator.update(gyro.getRotation2d(), getPositions()); 

    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
      est -> {
          var estPose = est.estimatedPose.toPose2d();
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = vision.getEstimationStdDevs(estPose);

          addVisionMeasurement(
                  est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      });

    // Log values to the dashboard
    log();
  }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
      poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public void addVisionMeasurement(
          Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose2d() { 
    return odometry.getPoseMeters(); 
  }

  public void log() {
    String table = "Drive/";
    Pose2d pose = getPose();
    SmartDashboard.putNumber(table + "X", pose.getX());
    SmartDashboard.putNumber(table + "Y", pose.getY());
    SmartDashboard.putNumber(table + "Heading", pose.getRotation().getDegrees());
    ChassisSpeeds chassisSpeeds = getSpeeds();
    SmartDashboard.putNumber(table + "VX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(table + "VY", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
            table + "Omega Degrees", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
    SmartDashboard.putNumber(table + "Target VX", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber(table + "Target VY", targetChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
            table + "Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
}

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 4.5);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Basic simulation of a swerve module, will just hold its current state and not use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Basic simulation of a gyro, will just hold its current state and not use any hardware
   */
  class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void updateRotation(double angularVelRps){
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
  }
}