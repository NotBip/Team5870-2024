package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import static frc.robot.Robot.m_robotContainer;

// import static frc.robot.util.MotorUtils.stopAllMotors;

public class Commands {
	// Vibrate
	public static final Command vibrateDriver = new StartEndCommand(
		() -> {
			m_robotContainer.driver.setRumble(RumbleType.kLeftRumble, 1.0);
			m_robotContainer.driver.setRumble(RumbleType.kRightRumble, 1.0);
		},
		() -> {
			m_robotContainer.driver.setRumble(RumbleType.kLeftRumble, 0.0);
			m_robotContainer.driver.setRumble(RumbleType.kRightRumble, 0.0);
		}
	).withTimeout(1.0);

	// public static final Command vibrateOperator = new StartEndCommand(
	// 	() -> {
	// 		m_robotContainer.operator.setRumble(RumbleType.kLeftRumble, 1.0);
	// 		m_robotContainer.operator.setRumble(RumbleType.kRightRumble, 1.0);
	// 	},
	// 	() -> {
	// 		m_robotContainer.operator.setRumble(RumbleType.kLeftRumble, 0.0);
	// 		m_robotContainer.operator.setRumble(RumbleType.kRightRumble, 0.0);
	// 	}
	// ).withTimeout(1.0);

	// public static final Command vibrateBoth = new ParallelCommandGroup(vibrateDriver, vibrateOperator);

	// Drive Train
	// public static final CommandBase emergencyStop = new InstantCommand(() -> { stopAllMotors(); }, Robot.driveTrain);
}
 