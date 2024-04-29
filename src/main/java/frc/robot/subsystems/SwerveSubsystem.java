package frc.robot.subsystems;


import java.sql.Driver;
import java.sql.Struct;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {
    
    private Field2d field = new Field2d(); 
    private AHRS navx = new AHRS(SPI.Port.kMXP);
    public SwerveModule[] SwerveMods;
    private SwerveDriveOdometry odometer; 
    public double simGyroRot = 0; 
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Module States", SwerveModuleState.struct).publish();

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                 zeroHeading();
            } catch (Exception e){}
        }).start();

            SwerveMods = new SwerveModule[] {
                new SwerveModule(
                    0,
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveEncoderReversed,
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed),
                new SwerveModule(
                    1,
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed),
                new SwerveModule(
                    2,
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveEncoderReversed,
                    DriveConstants.kBackLeftTurningEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed),
                new SwerveModule(
                    3,
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveEncoderReversed,
                    DriveConstants.kBackRightTurningEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed)
        };

        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());


        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            AutoConstants.pathFollowerConfig, 
            () -> { 
                var alliance = DriverStation.getAlliance(); 
                if(alliance.get() == DriverStation.Alliance.Red)
                    return true; 
                else
                    return false;
            }, this);

        SmartDashboard.putNumber("SimGyroRot", 0); 
        SmartDashboard.putBoolean("Zeroed Gyro", false); 

    }


    public void zeroHeading() {
        navx.reset(); 
    }


    public double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());

    }
    
    // public void setGyroforAuto() {  
    //     navx.setAngleAdjustment(-PathPlannerAuto.getStaringPoseFromAutoFile("Amp1").getRotation().getDegrees());
    // }

    public Pose2d getPose() { 
        return odometer.getPoseMeters(); 
    }

    public void resetOdometry(Pose2d pose) { 
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());

        SmartDashboard.putString("Robot Pose", getPose().toString());
        
        double p = SmartDashboard.getNumber("P Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        SmartDashboard.putNumber("gyro", getRotation2d().getRadians()); 
        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().toString()); 

        publisher.set(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getPositions();
        }
        return positions;
    }

    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] positions = new SwerveModuleState[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getState();
        }
        return positions;
    }


    public void stopModules() {
        SwerveMods[0].stop();
        SwerveMods[1].stop();
        SwerveMods[2].stop();
        SwerveMods[3].stop();
    }


    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveMods[0].setDesiredState(desiredStates[0], "Front Left");
        SwerveMods[1].setDesiredState(desiredStates[1], "Front Right");
        SwerveMods[2].setDesiredState(desiredStates[2], "Back Left");
        SwerveMods[3].setDesiredState(desiredStates[3], "Back Right");
        SmartDashboard.putNumber("Wheel Speeds", desiredStates[0].speedMetersPerSecond); 
    }

    public void getAbsoluteEncoder() { 
        SmartDashboard.putNumber("Front Left", SwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front RIght", SwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Left", SwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Right", SwerveMods[3].getAbsoluteEncoderRad());
    }

    public ChassisSpeeds getSpeeds() { 
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-robotRelativeSpeeds.vxMetersPerSecond, -robotRelativeSpeeds.vyMetersPerSecond, -robotRelativeSpeeds.omegaRadiansPerSecond, getRotation2d());
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02); 

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(targetStates); 
    }

    public void aprilTagMode(PhotonLL cam) { 
        SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond); 
        SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond); 
        SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond); 

        PIDController drivePID = new PIDController(0.05, 0, 0); 
        PIDController stafePID = new PIDController(0.05, 0, 0); 
        PIDController rotPID = new PIDController(0.1, 0, 0); 
        double driveOffset = 7; 
        double strafeOffset = 0; 
        double rotOffset = 0; 

        double velForward = drivePID.calculate(cam.getXDistance(), driveOffset);
        double velStrafe = stafePID.calculate(cam.getYDistance(), strafeOffset); 
        double rot = rotPID.calculate(cam.getYaw(), rotOffset); 


        velForward = xLimiter.calculate(velForward) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; 
        velStrafe = yLimiter.calculate(velStrafe) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; 
        rot = rotLimiter.calculate(rot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds; 

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(velForward, velStrafe, rot, getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

} // end Class
 
