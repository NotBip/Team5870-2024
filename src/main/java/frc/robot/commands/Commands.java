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
		
	// Ball delivery
	public static final Command launch = new RunCommand(() -> Robot.ballDelivery.power(Robot.io.operatorController.getRightY()), Robot.ballDelivery);

	public static final Command ballDeliveryReset = new InstantCommand(() -> {
		Robot.ballDelivery.raise();
		Robot.ballDelivery.stop();
	}, Robot.ballDelivery);

	public static final Command toggleBallDeliveryPistons = new InstantCommand(Robot.ballDelivery::togglePistons, Robot.ballDelivery);

	// Hatch Delivery
	public static final Command hatchDeliveryReset = new StartEndCommand(() -> {
		Robot.hatchDelivery.retract();
		Robot.hatchDelivery.closeGrabber();
	}, () -> {
		Robot.hatchDelivery.floatExtender();
		CommandScheduler.getInstance().schedule(vibrateBoth);
	}, Robot.hatchDelivery).withTimeout(1.0);
	
	public static final Command grab = new SequentialCommandGroup(
		new InstantCommand(Robot.hatchDelivery::closeGrabber, Robot.hatchDelivery),
		new InstantCommand(Robot.hatchDelivery::extend, Robot.hatchDelivery),
		new WaitUntilCommand(Robot.hatchDelivery::getLimitSwitch),
		new InstantCommand(Robot.hatchDelivery::floatExtender),
		new WaitUntilCommand(() -> !Robot.hatchDelivery.getLimitSwitch()),
		new InstantCommand(Robot.hatchDelivery::openGrabber),
		new WaitCommand(0.2),
		new InstantCommand(Robot.hatchDelivery::retract, Robot.hatchDelivery),
		new ScheduleCommand(vibrateBoth)
	);

	public static final Command release = new SequentialCommandGroup(
		new InstantCommand(Robot.hatchDelivery::openGrabber, Robot.hatchDelivery),
		new InstantCommand(Robot.hatchDelivery::extend, Robot.hatchDelivery),
		new WaitUntilCommand(Robot.hatchDelivery::getLimitSwitch),
		new InstantCommand(Robot.hatchDelivery::floatExtender),
		new WaitUntilCommand(() -> !Robot.hatchDelivery.getLimitSwitch()),
		new InstantCommand(Robot.hatchDelivery::closeGrabber),
		new WaitCommand(0.2),
		new InstantCommand(Robot.hatchDelivery::retract, Robot.hatchDelivery),
		new ScheduleCommand(vibrateBoth)
	);
		
}
 