package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * This command is responsible for resetting the climber's relative encoder positon. 
 */
public class zeroClimber extends Command {

    private Climber climber; // Define the subsystem

    public zeroClimber(Climber climber) { 
        this.climber = climber; // Initialize the subsystem. 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.resetEncoders(); // Reset the relative encoder for leader motor. 
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
