package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class IO {
    public final XboxController driverController = new XboxController(RobotMap.DRIVER_STICK_PORT);
    public final XboxController operatorController = new XboxController(RobotMap.OPERATOR_STICK_PORT);

    // JoystickButton grabButton = new JoystickButton(driverController, RobotMap.BTN_RIGHT_BUMPER);

    public IO() {
        // grabButton.onTrue(Robot.hatchDelivery.grab);
    }
}