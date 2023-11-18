package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
  
  // Motors for Driving 
  public static final CANSparkMax TOP_LEFT_MOTOR = new CANSparkMax(0, MotorType.kBrushless);
  public static final CANSparkMax TOP_RIGHT_MOTOR =  new CANSparkMax(2, MotorType.kBrushless); 
  public static final CANSparkMax BOTTOM_LEFT_MOTOR =  new CANSparkMax(4, MotorType.kBrushless); 
  public static final CANSparkMax BOTTOM_RIGHT_MOTOR =  new CANSparkMax(6, MotorType.kBrushless);  
  
  // Motors for Rotating
  public static final CANSparkMax TOP_LEFT_ROTMOTOR =  new CANSparkMax(1, MotorType.kBrushless); 
  public static final CANSparkMax TOP_RIGHT_ROTMOTOR =  new CANSparkMax(3, MotorType.kBrushless); 
  public static final CANSparkMax BOTTOM_LEFT_ROTMOTOR =  new CANSparkMax(5, MotorType.kBrushless); 
  public static final CANSparkMax BOTTOM_RIGHT_ROTMOTOR =  new CANSparkMax(7, MotorType.kBrushless);

  // Encoders for Driving
  public static final RelativeEncoder TOP_LEFT_MOTOR_BUILTIN_ENCODER = TOP_LEFT_MOTOR.getEncoder(); 
  public static final RelativeEncoder TOP_RIGHT_MOTOR_BUILTIN_ENCODER = TOP_RIGHT_MOTOR.getEncoder(); 
  public static final RelativeEncoder BOTTOM_LEFT_MOTOR_BUILTIN_ENCODER = BOTTOM_LEFT_MOTOR.getEncoder(); 
  public static final RelativeEncoder BOTTOM_RIGHT_MOTOR_BUILTIN_ENCODER = BOTTOM_RIGHT_MOTOR.getEncoder(); 
  
  // Encoders for Rotating
  public static final RelativeEncoder TOP_LEFT_ROTMOTOR_BUILTIN_ENCODER = TOP_LEFT_ROTMOTOR.getEncoder(); 
  public static final RelativeEncoder TOP_RIGHT_ROTMOTOR_BUILTIN_ENCODER = TOP_RIGHT_ROTMOTOR.getEncoder(); 
  public static final RelativeEncoder BOTTOM_LEFT_ROTMOTOR_BUILTIN_ENCODER = BOTTOM_LEFT_ROTMOTOR.getEncoder(); 
  public static final RelativeEncoder BOTTOM_RIGHT_ROTMOTOR_BUILTIN_ENCODER = BOTTOM_RIGHT_ROTMOTOR.getEncoder(); 

  public static final Encoder TOP_LEFT_ROTMOTOR_ENCODER = new Encoder(0, 1); 
  public static final Encoder TOP_RIGHT_ROTMOTOR_ENCODER = new Encoder(2, 3); 
  public static final Encoder BOTTOM_LEFT_ROTMOTOR_ENCODER = new Encoder(4, 5); 
  public static final Encoder BOTTOM_RIGHT_ROTMOTOR_ENCODER = new Encoder(6, 7); 

  // Driving Motor Groups
  public static final CANSparkMax[] LEFT_GROUP = { TOP_LEFT_MOTOR, BOTTOM_LEFT_MOTOR };
  public static final CANSparkMax[] RIGHT_GROUP = { TOP_RIGHT_MOTOR, BOTTOM_RIGHT_MOTOR };
  public static final CANSparkMax[] FRONT_GROUP = { TOP_LEFT_MOTOR, TOP_RIGHT_MOTOR };
  public static final CANSparkMax[] BACK_GROUP = { BOTTOM_LEFT_MOTOR, BOTTOM_RIGHT_MOTOR };

   // Rotating Motor Groups
  public static final CANSparkMax[] LEFT_ROT_GROUP = { TOP_LEFT_ROTMOTOR, BOTTOM_LEFT_ROTMOTOR };
  public static final CANSparkMax[] RIGHT_ROT_GROUP = { TOP_RIGHT_ROTMOTOR, BOTTOM_RIGHT_ROTMOTOR };
  public static final CANSparkMax[] FRONT_ROT_GROUP = { TOP_LEFT_ROTMOTOR, TOP_RIGHT_ROTMOTOR };
  public static final CANSparkMax[] BACK_ROT_GROUP = { BOTTOM_LEFT_ROTMOTOR, BOTTOM_RIGHT_ROTMOTOR };

  // ALL motor group
  public static final CANSparkMax[] ALL_MOTORS = { TOP_LEFT_MOTOR, TOP_LEFT_ROTMOTOR, TOP_RIGHT_MOTOR, TOP_RIGHT_ROTMOTOR, BOTTOM_LEFT_MOTOR, BOTTOM_LEFT_ROTMOTOR, BOTTOM_RIGHT_MOTOR, BOTTOM_RIGHT_ROTMOTOR };

  // Gyro
  public static final ADXRS450_Gyro GYRO = new ADXRS450_Gyro(SPI.Port.kOnboardCS2);


  // ======================================================== DRIVING MOTORS ==============================================  
  // Makes a motor controller group for all Driving motors. 
  public static final MotorControllerGroup allMotorGroup = new MotorControllerGroup(LEFT_GROUP[0], LEFT_GROUP[1], RIGHT_GROUP[0], RIGHT_GROUP[1]);

  // Makes a motor controller group for left and right motor group
  public static final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(LEFT_GROUP[0], LEFT_GROUP[1]);
  public static final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(RIGHT_GROUP[0], RIGHT_GROUP[1]);

  // Makes a motor controller group for Front and Back motor group
  public static final MotorControllerGroup frontMotorGroup = new MotorControllerGroup(FRONT_GROUP[0], FRONT_GROUP[1]);
  public static final MotorControllerGroup backMotorGroup = new MotorControllerGroup(BACK_GROUP[0], BACK_GROUP[1]);

  // ======================================================== ROTATING MOTORS ===============================================
  // Makes a motor controller group for all Rotating Motors
  public static final MotorControllerGroup allRotMotorGroup = new MotorControllerGroup(LEFT_ROT_GROUP[0], LEFT_ROT_GROUP[1], RIGHT_ROT_GROUP[0], RIGHT_ROT_GROUP[1]);

  // Makes a motor controller group for left and right motor group
  public static final MotorControllerGroup leftRotMotorGroup = new MotorControllerGroup(LEFT_ROT_GROUP[0], LEFT_ROT_GROUP[1]);
  public static final MotorControllerGroup rightRotMotorGroup = new MotorControllerGroup(RIGHT_ROT_GROUP[0], RIGHT_ROT_GROUP[1]);

  // Makes motor controller group for Front and Back motor group
  public static final MotorControllerGroup frontRotMotorGroup = new MotorControllerGroup(FRONT_ROT_GROUP[0], FRONT_ROT_GROUP[1]);
  public static final MotorControllerGroup backRotMotorGroup = new MotorControllerGroup(BACK_ROT_GROUP[0], BACK_ROT_GROUP[1]);

  // Controller
  public static final int DRIVER_STICK_PORT = 0;
  public static final int OPERATOR_STICK_PORT = 1;
}