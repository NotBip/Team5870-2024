package frc.robot.commands.Intake;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * This command is responsible for being able to take in live controller values from the joystick and set the speed for intake (CLASS USED TO SHOOT NOTES)
 */
public class LeftIntakeJoystick extends Command {

    private Intake intake; // Define the subsystem
    Supplier<Double> ySpdFunction; // Define the supplier variable to constantly provide us with the speed. 

    public LeftIntakeJoystick(Supplier<Double> ySpdFunction, Intake intake) { 
        this.intake = intake; // Initializing the subsystem
        this.ySpdFunction = ySpdFunction; // Initializing the supplier variable. 
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.IntakeMotorFront(-ySpdFunction.get()); // Supplying the speed to the motor by inverting it
        intake.IntakeMotorBack(ySpdFunction.get());  // Supplying the speed to the motor without any modifiers.
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeStop(); // Stop the motors once the command ends. 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
