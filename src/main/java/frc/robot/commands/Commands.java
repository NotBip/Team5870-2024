package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

public class Commands {
	// Drive Train
	public static final CommandBase emergencyStop = new InstantCommand(() -> Robot.driveTrain.stop(), Robot.driveTrain);
}
