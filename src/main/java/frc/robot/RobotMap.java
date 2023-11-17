package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class RobotMap {
  
  // Motors for Driving 
  public static final int TOP_LEFT_MOTOR = 0;
  public static final int TOP_RIGHT_MOTOR = 2; 
  public static final int BOTTOM_LEFT_MOTOR = 4; 
  public static final int BOTTOM_RIGHT_MOTOR = 6;  

  // Motors for Rotating
  public static final int TOP_LEFT_ROTMOTOR = 1; 
  public static final int TOP_RIGHT_ROTMOTOR = 3; 
  public static final int BOTTOM_LEFT_ROTMOTOR = 5; 
  public static final int BOTTOM_RIGHT_ROTMOTOR = 7;
  
  // Encoders for Rotating
  public static final int TOP_LEFT_ROTMOTOR_ENCODER = 0; 
  public static final int TOP_RIGHT_ROTMOTOR_ENCODER = 0; 
  public static final int BOTTOM_LEFT_ROTMOTOR_ENCODER = 0; 
  public static final int BOTTOM_RIGHT_ROTMOTOR_ENCODER = 0; 

  // Driving Motor Groups
  public static final Spark[] LEFT_GROUP = { new Spark(TOP_LEFT_MOTOR), new Spark(BOTTOM_LEFT_MOTOR) };
  public static final Spark[] RIGHT_GROUP = { new Spark(TOP_RIGHT_MOTOR), new Spark(BOTTOM_RIGHT_MOTOR) };
  public static final Spark[] FRONT_GROUP = { new Spark(TOP_LEFT_MOTOR), new Spark(TOP_RIGHT_MOTOR) };
  public static final Spark[] BACK_GROUP = { new Spark(BOTTOM_LEFT_MOTOR), new Spark(BOTTOM_RIGHT_MOTOR) };

   // Rotating Motor Groups
  public static final Spark[] LEFT_ROT_GROUP = { new Spark(TOP_LEFT_ROTMOTOR), new Spark(BOTTOM_LEFT_ROTMOTOR) };
  public static final Spark[] RIGHT_ROT_GROUP = { new Spark(TOP_RIGHT_ROTMOTOR), new Spark(BOTTOM_RIGHT_ROTMOTOR) };
  public static final Spark[] FRONT_ROT_GROUP = { new Spark(TOP_LEFT_ROTMOTOR), new Spark(TOP_RIGHT_ROTMOTOR) };
  public static final Spark[] BACK_ROT_GROUP = { new Spark(BOTTOM_LEFT_ROTMOTOR), new Spark(BOTTOM_RIGHT_ROTMOTOR) };


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


  public static final DifferentialDrive driveSystem = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  // Controller
  public static final int DRIVER_STICK_PORT = 0;
  public static final int OPERATOR_STICK_PORT = 1;
}