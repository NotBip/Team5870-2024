package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * This command is responsible for intaking notes by having the motors move at max speed. 
 */
public class IntakeSpinBack extends Command {

    private Intake intake; // Define the subsystem

    public IntakeSpinBack(Intake intake) { 
        this.intake = intake; // Initialize the subsystem
        addRequirements(intake); // Make it so this will only run when no other intake command is running
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Set the speeds for both intake motors. 
        intake.IntakeMotorFront(-1);
        intake.IntakeMotorBack(-1);    
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeStop(); // Stop the intake from moving.
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


}
