package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

public class Commands {
	// Vibrate
	public static final Command vibrateDriver = new StartEndCommand(
		() -> {
			Robot.io.driverController.setRumble(RumbleType.kLeftRumble, 1.0);
			Robot.io.driverController.setRumble(RumbleType.kRightRumble, 1.0);
		},
		() -> {
			Robot.io.driverController.setRumble(RumbleType.kLeftRumble, 0.0);
			Robot.io.driverController.setRumble(RumbleType.kRightRumble, 0.0);
		}
	).withTimeout(1.0);

	public static final Command vibrateOperator = new StartEndCommand(
		() -> {
			Robot.io.operatorController.setRumble(RumbleType.kLeftRumble, 1.0);
			Robot.io.operatorController.setRumble(RumbleType.kRightRumble, 1.0);
		},
		() -> {
			Robot.io.operatorController.setRumble(RumbleType.kLeftRumble, 0.0);
			Robot.io.operatorController.setRumble(RumbleType.kRightRumble, 0.0);
		}
	).withTimeout(1.0);

	public static final Command vibrateBoth = new ParallelCommandGroup(vibrateDriver, vibrateOperator);

	// Drive Train
	public static final CommandBase emergencyStop = new InstantCommand(() -> Robot.driveTrain.stop(), Robot.driveTrain);
}
 