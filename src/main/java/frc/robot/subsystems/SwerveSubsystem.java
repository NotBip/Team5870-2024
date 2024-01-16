package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;



public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule frontLeft; 

    private SwerveModule frontRight;

    private SwerveModule backLeft;

    private SwerveModule backRight;

    // Initializing Gyros. 
   // private ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS2);
      private AHRS navx = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){}
        }).start();

        new Thread(() -> {
            try{
                    frontLeft = new SwerveModule(
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveEncoderReversed,
                    DriveConstants.kFrontLeftTurningEncoderReversed,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
                    Thread.sleep(200);
                    frontRight = new SwerveModule(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveEncoderReversed,
                    DriveConstants.kFrontRightTurningEncoderReversed,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
                    Thread.sleep(200);
                    backLeft = new SwerveModule(
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveEncoderReversed,
                    DriveConstants.kBackLeftTurningEncoderReversed,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
                    Thread.sleep(200);
                    backRight = new SwerveModule(
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveEncoderReversed,
                    DriveConstants.kBackRightTurningEncoderReversed,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                    DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
                
            } catch (Exception e){}
        }).start();
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
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    /**
     * Method to get the rotation. 
     * @return Rotation2d in degrees. 
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Get Robot Heading in smart Dashboard. 
     */
    @Override
    public void periodic() {
    //    SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    /**
     * Stop al wheels/modules when called. 
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * sets the state for each modules/wheel.
     * @param desiredStates Array of state. 
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], "Front Left");
        frontRight.setDesiredState(desiredStates[1], "Front Right");
        backLeft.setDesiredState(desiredStates[2], "Back Left");
        backRight.setDesiredState(desiredStates[3], "Back Right");
    }
    
    // ============================================== METHODS FOR ZEROING WHEELS ===================================================

    public boolean zerofrontRight() { 
        if(frontRight.getAbsoluteEncoderRad() >= -0.009 && frontRight.getAbsoluteEncoderRad() <= 0.009) { 
            return false; 
        }
        else return true; 
    }
    
    public boolean zerofrontLeft() { 
        if(frontLeft.getAbsoluteEncoderRad() >= -0.009 && frontLeft.getAbsoluteEncoderRad() <= 0.009) { 
            return false; 
        }
        else return true; 
    }

    public boolean zerobackRight() { 
        if(backRight.getAbsoluteEncoderRad() >= -0.009 && backRight.getAbsoluteEncoderRad() <= 0.009) { 
            return false; 
        }
        else return true; 
    }

    public boolean zerobackLeft() { 
        if(backLeft.getAbsoluteEncoderRad() >= -0.009 && backLeft.getAbsoluteEncoderRad() <= 0.009) { 
            return false; 
        }
        else return true; 
    }



}
