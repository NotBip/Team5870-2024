package frc.robot.commands.Climber;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDownControllable extends Command {
    private Climber climber; 
    Supplier<Double> SpdFunction; 
    public ClimberDownControllable(Climber climber, Supplier<Double> SpdFunction) { 
        this.climber = climber; 
        this.SpdFunction = SpdFunction; 
        addRequirements(climber);
    }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {}
 
 
     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
        double spd = Math.abs(this.SpdFunction.get()) > 0.1 ? -this.SpdFunction.get() : 0.0; 
        climber.moveArmControllable(spd); 
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
