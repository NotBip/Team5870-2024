package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class LeftIntakeJoystick extends Command {
    private Intake intake; 
    private double ySpdFunction; 

    public LeftIntakeJoystick(double ySpdFunction, Intake intake) { 
        this.intake = intake; // setting intake to intake. 
        this.ySpdFunction = ySpdFunction; // the y soeed functuion
        addRequirements(intake); // addRequirements(intake)semicolon
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {} // initializes m


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.moveSpin(-ySpdFunction);
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
