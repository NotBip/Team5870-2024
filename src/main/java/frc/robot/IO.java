/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import frc.GodLIB.Controller;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class IO {

  public XboxController operatorController = new XboxController(RobotMap.OPERATOR_STICK_PORT);
  public XboxController driverController = new XboxController(RobotMap.DRIVER_STICK_PORT);

  public IO() {
    // operatorController.registerButtonEvent(Button.kA, true, new InstantCommand(() -> driverController.rumble(1, true, true)));
    // operatorController.registerButtonEvent(Button.kA, false, new InstantCommand(() -> driverController.endRumble()));
  }
}
