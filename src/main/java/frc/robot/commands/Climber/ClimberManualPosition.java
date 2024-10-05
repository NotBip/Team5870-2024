package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * This command is responsible to move the climber to a set position relative to the start position of the climber (ZERO THE CLIMBER AT A SET SPOT)
 */
public class ClimberManualPosition extends Command {

    private Climber climber; // Define the subsystem
    private double setRotation; // Define the variable to store the setpoint.

    public ClimberManualPosition(Climber climber, double setRotation) { 
        this.climber = climber; // Initialize the subsystem. 
        this.setRotation = setRotation; // Assign the setpoint to the variable.
        addRequirements(climber); // Makes it so this will only run if no other commands for climber is currently running
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.setPosition(setRotation); // Set the climber's position to the target setpoint.
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.hold(); // When the command ends set the motors to brake mode. 
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
 

    
}
