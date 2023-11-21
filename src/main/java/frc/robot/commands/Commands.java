package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import static frc.robot.Robot.io;

public class Commands {
	// Drive Train
	public static final CommandBase emergencyStop = new InstantCommand(() -> Robot.driveTrain.stop(), Robot.driveTrain);

}
 