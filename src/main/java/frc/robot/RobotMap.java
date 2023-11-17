/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
  public static final int TOP_RIGHT_MOTOR = 0; 
  public static final int BOTTOM_LEFT_MOTOR = 0; 
  public static final int BOTTOM_RIGHT_MOTOR = 0;  

  // Motors for Rotating
  public static final int TOP_LEFT_ROTMOTOR = 0; 
  public static final int TOP_RIGHT_ROTMOTOR = 0; 
  public static final int BOTTOM_LEFT_ROTMOTOR = 0; 
  public static final int BOTTOM_RIGHT_ROTMOTOR = 0; 

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

  // Button Declarations
  public static final int BTN_A = 1;
  public static final int BTN_B = 2;
  public static final int BTN_X = 3;
  public static final int BTN_Y = 4;
  public static final int BTN_LEFT_BUMPER = 5;
  public static final int BTN_RIGHT_BUMPER = 6;
  public static final int BTN_SELECT = 7;
  public static final int BTN_START = 8;
  public static final int BTN_LEFT_JOY_PRESS = 9;
  public static final int BTN_RIGHT_JOY_PRESS = 10;

  // Controller
  public static final int OPERATOR_STICK_PORT = 0;
  public static final int DRIVER_STICK_PORT = 2;

  // Sensors
  public static final int HD_LIMIT_SWITCH_PORT = 0; // Limit switch for hatch delivery
  public static final int BD_LEAF_SWITCH_PORT = 1; // Leaf switch for ball delivery
  
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = l1;
}