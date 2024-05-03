package frc.robot.commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RightIntakeJoystick extends Command {
    private Intake intake; 
    Supplier<Double> ySpdFunction; 

    public RightIntakeJoystick(Supplier<Double> ySpdFunction, Intake intake) { 
        this.intake = intake; // setting intake to intake. 
        this.ySpdFunction = ySpdFunction; // the y soeed functuion
        // addRequirements(intake); // addRequirements(intake)semicolon
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {} // initializes m


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.IntakeMotorBack(-Math.abs(ySpdFunction.get()));
        intake.IntakeMotorFront(Math.abs(ySpdFunction.get()*.7));
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeStop();

    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
    
}
