package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;



import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotMap {
    public static final int[] LEFT_GROUP = { 3, 4 };
    public static final int[] RIGHT_GROUP = { 1, 2};
    public static final int[] LEFT_ENCODERS = { 5, 6 };
    public static final int[] RIGHT_ENCODERS = { 8, 9};
    public static final boolean LeftEncoderReversed = true;
    public static final boolean RightEncoderReversed = false;
  //  public static final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(new MotorController[]{new PWMSparkMax(LEFT_GROUP[0]), new PWMSparkMax(LEFT_GROUP[1])});
    //    public static final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(new MotorController[]{new PWMSparkMax(RIGHT_GROUP[0]), new PWMSparkMax(RIGHT_GROUP[1])});
    public static final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(new MotorController[]{new CANSparkMax(LEFT_GROUP[0],MotorType.kBrushless), new CANSparkMax(LEFT_GROUP[1],MotorType.kBrushless)});

    public static final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(new MotorController[]{new CANSparkMax(RIGHT_GROUP[0],MotorType.kBrushless), new CANSparkMax(RIGHT_GROUP[1],MotorType.kBrushless)});
    public static final DifferentialDrive driveSystem = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    public static final int DRIVER_STICK_PORT = 0;
    public static final int OPERATOR_STICK_PORT = 1;
   
    public static final Encoder leftEncoder = new Encoder(LEFT_ENCODERS[0], LEFT_ENCODERS[1], LeftEncoderReversed);
    public static final Encoder rightEncoder = new Encoder(RIGHT_ENCODERS[0], RIGHT_ENCODERS[1], RightEncoderReversed);
    public static final ADXRS450_Gyro k_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS2);
    public final static Compressor c = new Compressor(0,PneumaticsModuleType.CTREPCM);
    public static final Solenoid solenoidUpDown1 = new Solenoid(PneumaticsModuleType.CTREPCM,1);
    public static final Solenoid solenoidUpDown2 = new Solenoid(PneumaticsModuleType.CTREPCM,2);
    public static final Solenoid solenoidInOut1 = new Solenoid(PneumaticsModuleType.CTREPCM,0);
    public static final Solenoid solenoidInOut2 = new Solenoid(PneumaticsModuleType.CTREPCM,5);
    public static final Solenoid solenoidOpenClose1 = new Solenoid(PneumaticsModuleType.CTREPCM,3);
    public static final Solenoid solenoidOpenClose2 = new Solenoid(PneumaticsModuleType.CTREPCM,4);


    public static boolean compressorOn =false;

    public static XboxController driverController = new XboxController(DRIVER_STICK_PORT);
    public static XboxController operatorController = new XboxController(OPERATOR_STICK_PORT);
   


    public static final double p = 0.03;
    //constants
    public static final double ksVolts = 1.4893;
    public static final double kvVoltSecondsPerMeter = 4.4441;
    public static final double kaVoltSecondsSquaredPerMeter = 1.8858;
    public static final double kPDriveVel = 5.1175;
    public static final double kTrackwidthMeters = 0.48;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static double slowModeMultiplier;
    public static final int kEncoderCPR = 1440; // (counts per revolution) default value
    public static final double kWheelDiameterMeters = 0.19431;  
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 0.06;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double forwardP= -0.012;
    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per seconds
    public static final int startingPosition = 1;
    public static final int autoMode = 1; // a number from 1 - 6

    //1 is on the left, 2 is in the middle and 3 is on the right
}
/*
 * java.base/java.lang.Thread.run(Unknown Source)
HAL: Incompatible State: The operation cannot be completed
Error at java.base/java.lang.Thread.run(Unknown Source): HAL: Incompatible State: The operation cannot be completed
edu.wpi.first.hal.SPIJNI.spiReadAutoReceivedData(Native Method)
edu.wpi.first.wpilibj.SPI$Accumulator.update(SPI.java:527)
edu.wpi.first.wpilibj.Notifier.lambda$new$0(Notifier.java:118)
java.base/java.lang.Thread.run(Unknown Source)d
HAL: Incompatible State: The operation cannot be completed
Error at java.base/java.lang.Thread.run(Unknown Source): HAL: Incompatible State: The operation cannot be completed
edu.wpi.first.hal.SPIJNI.spiReadAutoReceivedData(Native Method)
edu.wpi.first.wpilibj.SPI$Accumulator.update(SPI.java:527)
edu.wpi.first.wpilibj.Notifier.lambda$new$0(Notifier.java:118)
java.base/java.lang.Thread.run(Unknown Source)
 */