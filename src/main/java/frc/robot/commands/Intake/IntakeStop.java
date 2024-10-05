package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * This command is responsible to stop all intake motors. 
 */
public class IntakeStop extends Command {

    private Intake intake; // Define the subsystem

    public IntakeStop(Intake intake) { 
        this.intake = intake; // Initialize the subsystem
    }

    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.intakeStop(); // Stop the intake from moving.
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
