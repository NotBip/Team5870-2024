package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;



public class SwerveSubsystem extends SubsystemBase {
    
    // public final SwerveModule frontLeft = new SwerveModule(
    //     DriveConstants.kFrontLeftDriveMotorPort,
    //     DriveConstants.kFrontLeftTurningMotorPort,
    //     DriveConstants.kFrontLeftDriveEncoderReversed,
    //     DriveConstants.kFrontLeftTurningEncoderReversed,
    //     DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    //     DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
    //     DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    // public final SwerveModule frontRight = new SwerveModule(
    //     DriveConstants.kFrontRightDriveMotorPort,
    //     DriveConstants.kFrontRightTurningMotorPort,
    //     DriveConstants.kFrontRightDriveEncoderReversed,
    //     DriveConstants.kFrontRightTurningEncoderReversed,
    //     DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    //     DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
    //     DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    // public final SwerveModule backLeft = new SwerveModule(
    //     DriveConstants.kBackLeftDriveMotorPort,
    //     DriveConstants.kBackLeftTurningMotorPort,
    //     DriveConstants.kBackLeftDriveEncoderReversed,
    //     DriveConstants.kBackLeftTurningEncoderReversed,
    //     DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    //     DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    //     DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    // public final SwerveModule backRight = new SwerveModule(
    //     DriveConstants.kBackRightDriveMotorPort,
    //     DriveConstants.kBackRightTurningMotorPort,
    //     DriveConstants.kBackRightDriveEncoderReversed,
    //     DriveConstants.kBackRightTurningEncoderReversed,
    //     DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    //     DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
    //     DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
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

    }

    /**
     * Reset Gyro heading usually only during the initializing. 
     */
    public void zeroHeading() {
        navx.reset(); 
    }

    /**
     * Method to get the current heading of the gyro. Used for swerve
     * @return The Heading of the gyro in degrees.  
     */
    public double getHeading() {
        return Math.IEEEremainder(-navx.getAngle(), 360); 
    }

    /**
     * Method to get the rotation. 
     * @return Rotation2d in degrees. 
     */
    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(-navx.getFusedHeading());
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
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Magnet", navx.getFusedHeading()); 
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : SwerveMods){
             positions[mod.modNum] = mod.getPositions();
        }
        return positions;
    }

    /**
     * Stop al wheels/modules when called. 
     */
    public void stopModules() {
        SwerveMods[0].stop();
        SwerveMods[1].stop();
        SwerveMods[2].stop();
        SwerveMods[3].stop();
    }

    /**
     * sets the state for each modules/wheel.
     * @param desiredStates Array of state. 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveMods[0].setDesiredState(desiredStates[0], "Front Left");
        SwerveMods[1].setDesiredState(desiredStates[1], "Front Right");
        SwerveMods[2].setDesiredState(desiredStates[2], "Back Left");
        SwerveMods[3].setDesiredState(desiredStates[3], "Back Right");
        SmartDashboard.putNumber("LJKSAD", navx.getAngle());
    }

    public void Encoder() { 
        SmartDashboard.putNumber("Front Left", SwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front RIght", SwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Left", SwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Right", SwerveMods[3].getAbsoluteEncoderRad());
    }

    public void alignAprilTag(int id) { 
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(id);
        NetworkTableEntry tx = table.getEntry("tx"); // Limelight's x-coordinate of the target
        NetworkTableEntry ty = table.getEntry("ty"); // Limelight's y-coordinate of the target
        NetworkTableEntry ta = table.getEntry("ta"); // Limelight's area of the target
        NetworkTableEntry tv = table.getEntry("tv");
        ChassisSpeeds chassisSpeeds; 

        while (true) { 
            double y = ty.getDouble(0.0);
            double area = ta.getDouble(0.0);
            double x = tx.getDouble(0.0);
            
            // if (area == 0) { 
            //     chassisSpeeds = new ChassisSpeeds(1, 0, 0); 
            // }
            if (area < 80 && area > 0) { 
                chassisSpeeds = new ChassisSpeeds(1, 0, 0); 
                SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                setModuleStates(moduleStates);
            } 
            else { 
                stopModules();
                break; 
            }   
        }
    }

 }
