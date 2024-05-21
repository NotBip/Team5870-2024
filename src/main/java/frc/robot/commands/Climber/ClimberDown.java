package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDown extends Command {
    private Climber climber; 

    public ClimberDown(Climber climber) { 
        this.climber = climber; 
        addRequirements(climber);
    }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {}
 
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
        climber.moveArmControllable(-0.20); 
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
