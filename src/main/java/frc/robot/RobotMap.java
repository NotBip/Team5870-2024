package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
  // Controller
  public static final int DRIVER_STICK_PORT = 0;
  public static final int OPERATOR_STICK_PORT = 1;
  public static final double stickDeadband = 0.1;

  public static final class Swerve {
    // Spark Max Idle Modes
    public static final CANSparkMax.IdleMode driveIdleMode = CANSparkMax.IdleMode.kBrake;
    public static final CANSparkMax.IdleMode angleIdleMode = CANSparkMax.IdleMode.kBrake;

    // Max Output Powers
    public static final double drivePower = 1;
    public static final double anglePower = .9;

    // Gyro
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    // meters per rotation
    // ========================= NEED TO FIND ===============================
    public static final double driveGearRatio = 6.75; 
    public static final double angleGearRatio = 21.43; 
    public static final double wheelCircumference = 5;
    public static final double driveRevToMeters =  wheelCircumference / driveGearRatio;
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60 ;

    public static final boolean driveMotorInvert = false;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = false;

    // the number of degrees that a single rotation of the turn motor turns the wheel.
    public static final double DegreesPerTurnRotation = 360/angleGearRatio;

    /* Drivetrain Constants */ // ========================= NEED TO FIND ===============================
    public static final double trackWidth = Units.inchesToMeters(23.75);
    public static final double wheelBase = Units.inchesToMeters(23.75);
    
    // Max Output Powers
    public static final double maxDrivePower = 1;
    public static final double maxAnglePower = .9;

    /* Swerve Kinematics 
    * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;
    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.05;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKFF = 0;

    /* Drive Motor info  */
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference) / driveGearRatio;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.04;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 1 / kDriveWheelFreeSpeedRps;
    /** Meters per Second */
    public static final double maxSpeed = 3.6576;
    /** Radians per Second */
    public static final double maxAngularVelocity = 5.0;
    public static double angleRampRate = 0;

    // CANCoder CONFIG
    // public static final CANCoderConfiguration swerveCANcoderConfig = new CANCoderConfiguration();



    public static class Modules {
      /* Front Left Module - Module 0 */
      public static final class FLMod {

          public static final int driveMotorID = 0;
          public static final int angleMotorID = 1;
          public static final int canCoderID = 0;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.146-180); //Rotation2d.fromDegrees(37.7);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class FRMod {
          public static final int driveMotorID = 2;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 1;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.015);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module - Module 2 */
      public static final class BLMod {
          public static final int driveMotorID = 4;
          public static final int angleMotorID = 5;
          public static final int canCoderID = 2;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(263.603);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Modu */
      public static final class BRMod {
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 7;
          public static final int canCoderID = 3;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(317.021);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

    public static final double X_kP = 5;
    public static final double X_kI = 0;
    public static final double X_kD = 0;

    public static final double Y_kP = 5;
    public static final double Y_kI = 0;
    public static final double Y_kD = 0;

    public static final double THETA_kP = 6.2;
    public static final double THETA_kI = 0;
    public static final double THETA_kD = 0;


    // Motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularSpeedRadiansPerSecondSquared);
}



  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}