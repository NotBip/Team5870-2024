package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

/**
 * This command is responsible for extending the pistons out (THIS SHOULD BE ONLY CALLED A SINGLE TIME SO USE .onTrue() instead of .whileTrue).
 */
public class FullExtend extends Command {

    private Pneumatics pneumatics; // Define the subsystem

    public FullExtend(Pneumatics pneumatics) { 
        this.pneumatics = pneumatics; // Initialize the subsystem. 
        addRequirements(pneumatics); // // This command will only run if there aren't anything other commands already running for pneumatics subsystem. 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pneumatics.extendArm(); // Extend the arm
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
