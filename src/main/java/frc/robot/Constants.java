package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {


    // Module for Each Swerve Module.
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

    // Module for Intake Constants
    public static final class IntakeConstants { 
        public static double intakeSpeed = 0.5; 
        public static int armMotor1 = 9; 
        public static int armMotor2 = 8; 
    }

    public static final class PneumaticsConstants { 
        public static int solenoidExtend1ID = 0; 
        public static int solenoidExtend2ID = 2; 
        public static int solenoidDetract1ID = 1; 
        public static int solenoidDetract2ID = 4; 
        public static int compressorID = 0; 
    }

    // Module for Swerve Drive. 
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

        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kFrontRightDriveMotorPort = 6;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 7;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
        public static final int kBackRightDriveAbsoluteEncoderPort = 11;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.8748354701;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 5.91345937;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.84891327050;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 4.015961702; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }


    // Module for Autonomous Mode. 
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = .3;
        public static final double kPYController = .3;
        public static final double kPThetaController = .3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, 
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(0.07, 0, 0),
            new PIDConstants(.05, 0, 0.01),
            4.5, 
            0.5374011537, // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
            );
    }


    // Module for Controller Joystick
    public static final class OIConstants {

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

    public static final class climberConstants { 
        public static final int leaderMotor = 14; 
        public static final int followerMotor = 13;
        public static final int bottomLimitSwitch = 0; 
        public static final int topLimitSwitch = 0;  
    } 
}
