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

import frc.robot.util.DoubleSolenoidToggler;
import frc.robot.util.FloatableSolenoid;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class RobotMap {

  // Ball Shooter Motors
  public static final int LEFT_MOTOR = 3;
  public static final int RIGHT_MOTOR = 4;

  // Drive Train motors
  public static final int[] LEFT_GROUP = { 0, 1 };
  public static final int[] RIGHT_GROUP = { 8, 9 };

  public static final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(new Spark(RobotMap.LEFT_GROUP[0]),
    new Spark(RobotMap.LEFT_GROUP[1]));

  public static final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(new Spark(RobotMap.RIGHT_GROUP[0]),
    new Spark(RobotMap.RIGHT_GROUP[1]));

  public static final DifferentialDrive driveSystem = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  // Pneumatics
  public static final FloatableSolenoid extenderSolenoid = new FloatableSolenoid(2, 3, 4, 5);
  public static final DoubleSolenoidToggler grabberSolenoid = new DoubleSolenoidToggler(1, 0); 
  public static final DoubleSolenoidToggler pistons = new DoubleSolenoidToggler(6, 7);

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
  // public static int rangefinderModule = 1;
 
}