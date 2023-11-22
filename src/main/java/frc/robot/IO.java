package frc.robot;

import frc.GodLIB.Controller;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class IO {

  public Controller driverController = new Controller(RobotMap.DRIVER_STICK_PORT);
  public Controller operatorController = new Controller(RobotMap.OPERATOR_STICK_PORT);

  public IO() {
    // operatorController.registerButtonEvent(Button.kA, true, new InstantCommand(() -> driverController.rumble(1, true, true)));
    // operatorController.registerButtonEvent(Button.kA, false, new InstantCommand(() -> driverController.endRumble()));
  }
}
