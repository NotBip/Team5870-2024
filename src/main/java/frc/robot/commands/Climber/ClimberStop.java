package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * This command is responsible for braking and stopping the climber.
 */
public class ClimberStop extends Command {

    private Climber climber; // Define the subsystem

    public ClimberStop(Climber climber) { 
        this.climber = climber; // Initialize the subsystem. 
        addRequirements(climber); // Makes it so this will only run if no other commands for climber is currently running
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
 
 
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.hold(); // Brake and set the motor's speed to 0
    }
 
 
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
 
     
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


    
}
