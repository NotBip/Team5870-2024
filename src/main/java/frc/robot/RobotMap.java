package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
  // Gyro
  public static final ADXRS450_Gyro GYRO = new ADXRS450_Gyro(SPI.Port.kOnboardCS2);
  
  // Joystick DeadBands
  public static final double LeftStickDeadBand = 0.35; 
  public static final double RightStickDeadBand = 0.15; 

  // Controller
  public static final int DRIVER_STICK_PORT = 0;
  public static final int OPERATOR_STICK_PORT = 1;

  // Motor groups
  public static final CANSparkMax[] ALL_GROUP = { Modules.TL.drive, Modules.TL.rotate, Modules.TR.drive, Modules.TR.rotate, Modules.BL.drive, Modules.BL.rotate, Modules.BR.drive, Modules.BR.rotate };
  public static final MotorControllerGroup ALL_DRIVE_GROUP = new MotorControllerGroup(Modules.TL.drive, Modules.TR.drive, Modules.BL.drive, Modules.BR.drive);
  public static final MotorControllerGroup ALL_ROTATE_GROUP = new MotorControllerGroup(Modules.TL.rotate, Modules.TR.rotate, Modules.BL.rotate, Modules.BR.rotate);

  
  public static class Modules {
    public static class TL {
      public static CANSparkMax drive = new CANSparkMax(0, MotorType.kBrushless);
      public static CANSparkMax rotate = new CANSparkMax(1, MotorType.kBrushless);
      public static SparkMaxPIDController drivePID = drive.getPIDController();
      public static SparkMaxPIDController rotatePID = rotate.getPIDController();
      public static double p = rotatePID.getP();
      public static double i = rotatePID.getI();
      public static double d = rotatePID.getD();
      public static RelativeEncoder driveEncoder = drive.getEncoder(); 
      public static RelativeEncoder rotateEncoder = rotate.getEncoder(); 
      public static CANCoder hardwareRotateEncoder = new CANCoder(0);

      public static void driveAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
      public static void rotateAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
    }

    public static class TR {
      public static CANSparkMax drive = new CANSparkMax(2, MotorType.kBrushless);
      public static CANSparkMax rotate = new CANSparkMax(3, MotorType.kBrushless);
      public static SparkMaxPIDController drivePID = drive.getPIDController();
      public static SparkMaxPIDController rotatePID = rotate.getPIDController();
      public static double p = rotatePID.getP();
      public static double i = rotatePID.getI();
      public static double d = rotatePID.getD();
      public static RelativeEncoder driveEncoder = drive.getEncoder(); 
      public static RelativeEncoder rotateEncoder = rotate.getEncoder(); 
      public static CANCoder hardwareRotateEncoder = new CANCoder(1);

      public static void driveAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
      public static void rotateAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
    }

    public static class BL {
      public static CANSparkMax drive = new CANSparkMax(4, MotorType.kBrushless);
      public static CANSparkMax rotate = new CANSparkMax(5, MotorType.kBrushless);
      public static SparkMaxPIDController drivePID = drive.getPIDController();
      public static SparkMaxPIDController rotatePID = rotate.getPIDController();
      public static double p = rotatePID.getP();
      public static double i = rotatePID.getI();
      public static double d = rotatePID.getD();
      public static RelativeEncoder driveEncoder = drive.getEncoder(); 
      public static RelativeEncoder rotateEncoder = rotate.getEncoder(); 
      public static CANCoder hardwareRotateEncoder = new CANCoder(2);

      public static void driveAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
      public static void rotateAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
    }

    public static class BR {
      public static CANSparkMax drive = new CANSparkMax(6, MotorType.kBrushless);
      public static CANSparkMax rotate = new CANSparkMax(7, MotorType.kBrushless);
      public static SparkMaxPIDController drivePID = drive.getPIDController();
      public static SparkMaxPIDController rotatePID = rotate.getPIDController();
      public static double p = rotatePID.getP();
      public static double i = rotatePID.getI();
      public static double d = rotatePID.getD();
      public static RelativeEncoder driveEncoder = drive.getEncoder(); 
      public static RelativeEncoder rotateEncoder = rotate.getEncoder(); 
      public static CANCoder hardwareRotateEncoder = new CANCoder(3);

      public static void driveAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
      public static void rotateAngle(double angle) {
        rotatePID.setReference(angle, ControlType.kPosition);
      }
    }
  }
}
