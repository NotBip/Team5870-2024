/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import frc.robot.commands.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class IO {

  public XboxController operatorController = new XboxController(RobotMap.OPERATOR_STICK_PORT);
  public XboxController driverController = new XboxController(RobotMap.DRIVER_STICK_PORT);

  JoystickButton grabButton = new JoystickButton(operatorController, RobotMap.BTN_RIGHT_BUMPER);
  JoystickButton releaseButton = new JoystickButton(operatorController, RobotMap.BTN_LEFT_BUMPER);
  JoystickButton hatchResetButton = new JoystickButton(operatorController, RobotMap.BTN_START);

  JoystickButton pistonButton = new JoystickButton(operatorController, RobotMap.BTN_A);
  JoystickButton ballResetButton = new JoystickButton(operatorController, RobotMap.BTN_SELECT);


  public IO() {
    grabButton.onTrue(Commands.grab);
    releaseButton.onTrue(Commands.release);
    hatchResetButton.onTrue(Commands.hatchDeliveryReset);

    pistonButton.onTrue(Commands.toggleBallDeliveryPistons);
    ballResetButton.onTrue(Commands.ballDeliveryReset);
  }

}
