package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * This class is used to store all Constant variables. All the variables should be final and static.
 */
public final class Constants {


    /**
     *  Contains all the variables needed to configure each swerve module. 
     */ 
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0/7); 
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.2; 
        public static final double slowModeMultiplier = 0.5;           
    }

    /**
     * Contains all the motor ID's for the intake subsystem
     */
    public static final class IntakeConstants { 
        public static int armMotor1 = 9; 
        public static int armMotor2 = 8; 
    }

    /**
     * Contains all the solenoid ID's for the pneumatics subsystem. 
     */
    public static final class PneumaticsConstants { 
        public static int solenoidExtend1ID = 0; 
        public static int solenoidExtend2ID = 2; 
        public static int solenoidDetract1ID = 1; 
        public static int solenoidDetract2ID = 4; 
    }

    /**
     * Contains all the motor ID's for the climber subsystem. 
     */
    public static final class climberConstants { 
        public static final int leaderMotor = 14; 
        public static final int followerMotor = 13;
        public static final int bottomLimitSwitch = 0; 
        public static final int topLimitSwitch = 0;  
    } 

    /**
     * Contains all the variables needed to setup the swerve drive train. 
     */
    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(30);

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(30);
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            // front left
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
            // front right
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
            // back left
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), 
            // back right
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Drive Motor ID'S. 
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kFrontRightDriveMotorPort = 6;

        // Turning Motor ID'S
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 7;

        // Reversing Turning Encoder Values (Only change this if it is rotating the wrong way)
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        // Reverse Drive Encoder Values (Only change this if it is spinning the wrong way)
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // Absolute Encoder Port ID's
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
        public static final int kBackRightDriveAbsoluteEncoderPort = 11;

        // Reverse Absolute Encoder Values
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // Absolute Encoder offset (To Configure this set all values to zero and rotate all the wheels to the front and note down the values)
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.85029177;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.8378644;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.719592;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 3.946932567; 

        // Robot Speed Limits. 
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }


    /**
     * Contains all the variables needed to configure autonmous mode. 
     */
    public static final class AutoConstants {
        // Configuring Robot's autonmous settings for PathFollower
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(0.01, 0, 0),
            new PIDConstants(.01, 0, 0.01),
            5, 
            0.5374011537, // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
            );
    }


    /**
     * Contains all the Port and button ID's needed to setup the Driver/Operator Controller. 
     */
    public static final class IOConstants {

        // Joystick Values Used for Swerve Controls
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1; 
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final double kDeadband = 0.1;

        // Xbox Controller Map
        public static final int KXboxButtonA = 1; 
        public static final int KXboxButtonB = 2;
        public static final int KXboxButtonX = 3;  
        public static final int KXboxButtonY = 4; 
        public static final int KXboxLeftBumper = 5; 
        public static final int KXboxRightBumper = 6; 
        public static final int KXboxSelectButton = 7; 
        public static final int KXboxStartButton = 8; 
        public static final int KXboxLeftTrigger = 9;
        public static final int KXboxRightTrigger = 10; 
        
    }
}
