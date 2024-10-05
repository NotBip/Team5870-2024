package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

/**
 * Class used to configure and run the Swerve Drive train (Must extend SubsystemBase). 
 */
public class SwerveSubsystem extends SubsystemBase {
    
    private Field2d field = new Field2d(); // Generate a 2D field to be used in smartDashboard later. 
    private AHRS navx = new AHRS(SPI.Port.kMXP); // Gyro used to keep track of the robot's current heading (angle). 
    public SwerveModule[] SwerveMods; // Making an array of the SwerveMods for a 4 wheel swerve drive. 
    private SwerveDriveOdometry odometer; // Odometer used to keep track of the position of the robot on the field. 

    public SwerveSubsystem(){

        // 1 Second after Initializing this class it will zero the heading reseting the absolute front position of the robot. 
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                 zeroHeading();
            } catch (Exception e){}
        }).start();

            // Initalizing the array of Swerve Modules with the proper port ID's for each Swerve module. [FL, FR, BL, BR]. 
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

        odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, new Rotation2d(0), getModulePositions()); // Initialize the odometer with it's initial values. 

        // Configure the autonmous builder for pathPlanner. 
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            AutoConstants.pathFollowerConfig, // Flip the autonmous path when changing alliance. 
            () -> { 
                var alliance = DriverStation.getAlliance(); 
                if(alliance.get() == DriverStation.Alliance.Red)
                    return true; 
                else
                    return false;
            }, this);
    }


    /**
     * This method is used the reset the gyro on the robot. Resetting it's absolute front position. 
     */
    public void zeroHeading() {
        navx.reset(); 
    }

    /**
     * Method used to calculate the current heading of the robot (CCW Positive, 0 to 360) in degrees
     * @return a double value which contains the current heading of the robot from 0 to 360 in degrees. 
     */
    public double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360); 
    }

    /** 
     * Converts the heading to Rotation2D. 
     * @return The Robot's current heading in Rotation2D. 
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());

    }

    /**
     * Uses the odometer to get the current x and y position of the robot. 
     * @return The current x and y position of the robot on the field based on the odometer in Pose2D
     */
    public Pose2d getPose() { 
        return odometer.getPoseMeters(); 
    }

    /**
     * Sets the odometer of the robot to whatever Pose2D you want. Typically used at the start of autonomous mode. 
     * @param pose The new pose2D of the robot. 
     */
    public void resetOdometry(Pose2d pose) { 
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    // This method runs periodically the whole time this class has been initalized. 
    @Override
    public void periodic() {
        field.setRobotPose(getPose()); // set the position of the robot in the 2D field generated for SmartDashboard. 
        odometer.update(getRotation2d(), getModulePositions()); // constantly update the odometer with the current rotation and position of each module. 
    }

    /**
     * Goes through each of the Swerve Module to return their current positon. 
     * @return The current position of each swerve module in an array [FL, FR, BL, BR]. 
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getPositions();
        }
        return positions;
    }

    /**
     * Goes through each of the Swerve Module to return their current state. 
     * @return The current state of each swerve module in an array [FL, FR, BL, BR].
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] state = new SwerveModuleState[4];
        for(SwerveModule mod : SwerveMods){
            state[mod.modNum] = mod.getState();
        }
        return state;
    }

    /**
     * This method stops all the swerve modules by setting both their drive and turning motor speeds to 0
     */
    public void stopModules() {
        SwerveMods[0].stop();
        SwerveMods[1].stop();
        SwerveMods[2].stop();
        SwerveMods[3].stop();
    }

    /**
     * This method puts a limit on the max speed for each swerve module and sets the desired state after. 
     * @param desiredStates The state of each swerve module using the SwerveModuleState class [FL, FR, BL, BR].
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // Limits the speed 
        SwerveMods[0].setDesiredState(desiredStates[0], "Front Left");
        SwerveMods[1].setDesiredState(desiredStates[1], "Front Right");
        SwerveMods[2].setDesiredState(desiredStates[2], "Back Left");
        SwerveMods[3].setDesiredState(desiredStates[3], "Back Right");
    }

    /**
     * Returns the absolute encoder value for each swerve module. This method is used initially to calibrate the front of each absolute encoder. 
     */
    public void getAbsoluteEncoder() { 
        SmartDashboard.putNumber("Front Left", SwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front RIght", SwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Left", SwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Right", SwerveMods[3].getAbsoluteEncoderRad());
    }

    /**
     * This method uses modules states and converts them to the overall chassis speed uses Swerve Drive Kinematics. 
     * @return The speed of the Chassis. 
     */
    public ChassisSpeeds getSpeeds() { 
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    /**
     * This method sets the target states for each module using field relative speeds. (Robot can turn and drive at the same time). This method is to be 
     * mainly used in the autoBuilder.  
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-robotRelativeSpeeds.vxMetersPerSecond, -robotRelativeSpeeds.vyMetersPerSecond, -robotRelativeSpeeds.omegaRadiansPerSecond, getRotation2d());
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02); 

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(targetStates); 
    }

} // end Class
 
