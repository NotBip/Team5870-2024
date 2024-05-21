package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ClimberManualPosition extends Command {
    private Climber climber; 
    private double setRotation; 

    public ClimberManualPosition(Climber climber, double setRotation) { 
        this.climber = climber; 
        this.setRotation = setRotation; 
        addRequirements(climber);
    }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {}
 
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
        climber.setPosition(setRotation); 
     }
 
 
     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
         climber.hold();
     }
 
     
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return false;
     }
 

    
}
