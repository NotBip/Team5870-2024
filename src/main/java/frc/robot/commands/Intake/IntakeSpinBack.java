package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSpinBack extends Command {

    private Intake intake; 

    public IntakeSpinBack(Intake intake) { 
        this.intake = intake; 
        addRequirements(intake);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.IntakeMotorFront(-1);
        intake.IntakeMotorBack(-1);    
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
