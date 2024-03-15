package frc.robot.subsystems;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
                if(alliance.isPresent()) { 
                    return alliance.get() == DriverStation.Alliance.Red; 
                }
                return false;
            }, this);
    }


    public void zeroHeading() {
        navx.reset(); 
    }


    public double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360); 
    }

    public Rotation2d getRotation2d() {
         return Rotation2d.fromDegrees(-navx.getYaw());

    }

    public Pose2d getPose() { 
        return odometer.getPoseMeters(); 
    }

    public void resetOdometry(Pose2d pose) { 
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
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
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02); 

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates); 
    }

    public void alignAprilTag() { 
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tableID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); 
        table.getEntry("pipeline").setNumber(tableID);
        NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
        NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
        NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
        NetworkTableEntry tv = table.getEntry("tv");
        double prevarea = ta.getDouble(0); 
        double prevX = tx.getDouble(0); 
        double prevY = ty.getDouble(0); 
        
        boolean rotDone = false; 
        boolean xDone = false; 
        boolean yDone = false; 
        ChassisSpeeds chassisSpeeds; 

        // determine distance to goal
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double mountAngleDegrees = 30.0;
        double lensHeight = 33.02;
        double goalHeight = 121.9;
        double angleToGoalRadians = (mountAngleDegrees + targetOffsetAngle_Vertical) * (Math.PI/180);
        double distLightToGoal = (goalHeight-lensHeight) / Math.tan(angleToGoalRadians);

        switch ((int) tableID) {
            case 6:
                while (true) { 
                    SmartDashboard.putBoolean("ID DETECTED", true); 
                    SmartDashboard.putBoolean("xDone", xDone);
                    SmartDashboard.putBoolean("yDone", yDone);
                    SmartDashboard.putBoolean("rotDone", rotDone);

                    double y = ty.getDouble(0.0);
                    double x = tx.getDouble(0.0);
                    double area = ta.getDouble(0.0); 

                    System.out.println(prevX);
                    // Rotating Towards The Goal. 
                    if ((getRotation2d().getDegrees() > 0.5 || getRotation2d().getDegrees() < -0.5) && !rotDone) { 
                        if (getRotation2d().getDegrees() > 0.5) { 
                            chassisSpeeds = new ChassisSpeeds(0, 0, getRotation2d().getDegrees()*-1); 
                            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                            setModuleStates(moduleStates);
                        } else if (getRotation2d().getDegrees() < -0.5) { 
                            chassisSpeeds = new ChassisSpeeds(0, 0, getRotation2d().getDegrees()*-1); 
                            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                            setModuleStates(moduleStates); 
                        }
                    } else if (getRotation2d().getDegrees() < 0.5 && getRotation2d().getDegrees() > -0.5 && !rotDone || x == 0 ) { 
                        stopModules();
                        rotDone = true; 
                        break; 
                    }
                }   
            default: 
                System.out.println("ASdasda");
                    
        }
    }

} // end Class
 
