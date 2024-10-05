package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * This command is responsible for moving the climber down at a constants speed of .20 (Speed is recommended to be below .4)
 */
public class ClimberDown extends Command {

    private Climber climber; // Define the subsystem

    public ClimberDown(Climber climber) { 
        this.climber = climber; // Initialize the subsystem. 
        addRequirements(climber); // Makes it so this will only run if no other commands for climber is currently running
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.moveArmControllable(0.20); // Sets the climber speed to -.20 to move it down.
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
