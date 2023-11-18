package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public static final class Swerve {
    // meters per rotation
    // ========================= NEED TO FIND ===============================
    public static final double driveGearRatio = 0; 
    public static final double wheelCircumference = 5;
    public static final double driveRevToMeters =  wheelCircumference / driveGearRatio;
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60 ;

    /* Drivetrain Constants */ // ========================= NEED TO FIND ===============================
    public static final double trackWidth = Units.inchesToMeters(23.75);
    public static final double wheelBase = Units.inchesToMeters(23.75);

    // Max Output Powers
    public static final double maxDrivePower = 1;
    public static final double maxAnglePower = .9;

    /* Drive Motor info  */
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference) / driveGearRatio;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.04;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 1 / kDriveWheelFreeSpeedRps;
    


    public static class Modules {
      /* Front Left Module - Module 0 */
      public static final class Mod0 {

          public static final int driveMotorID = 0;
          public static final int angleMotorID = 1;
          public static final int canCoderID = 0;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.146-180); //Rotation2d.fromDegrees(37.7);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 {
          public static final int driveMotorID = 2;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 1;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(26.015);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module - Module 2 */
      public static final class Mod2 {
          public static final int driveMotorID = 4;
          public static final int angleMotorID = 5;
          public static final int canCoderID = 2;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(263.603);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 {
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 7;
          public static final int canCoderID = 3;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(317.021);
          public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}