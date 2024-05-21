package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeFullPower extends Command {
    private Intake intake; 

    public IntakeFullPower(Intake intake) { 
        this.intake = intake; 
        addRequirements(intake); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.moveSpin(-1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.moveSpin(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }



}
