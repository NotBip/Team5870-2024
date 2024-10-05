package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This command is responsible for reseting the robot's heading to 0 changing it's absolute front positon.
 */
public class ZeroGyro extends Command {

    private SwerveSubsystem swerveSubsystem; // Define the subsystem
    
    public ZeroGyro(SwerveSubsystem swerveSubsystem) { 
        this.swerveSubsystem = swerveSubsystem; // Initialize the subsystem
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSubsystem.zeroHeading(); // Zeroes the gyro's heading. 
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
    
}
