package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

public class FullDetract extends Command {
    private Pneumatics pneumatics; 

    public FullDetract(Pneumatics pneumatics) { 
        this.pneumatics = pneumatics; 
        addRequirements(pneumatics); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pneumatics.detractArm(); 
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
