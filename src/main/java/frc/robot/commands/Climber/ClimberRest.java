package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberRest extends Command {
    private Climber climber; 

    public ClimberRest(Climber climber) { 
        this.climber = climber; 
        addRequirements(climber);
    }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {}
 
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
        climber.resetEncoders();
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
