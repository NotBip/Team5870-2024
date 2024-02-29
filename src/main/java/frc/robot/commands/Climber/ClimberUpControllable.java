package frc.robot.commands.Climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberUpControllable extends Command {
    private Climber climber; 
    Supplier<Double> SpdFunction;

    public ClimberUpControllable(Climber climber, Supplier<Double> SpdFunction) { 
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
        double spd = this.SpdFunction.get(); 
        climber.moveArmControlleble(spd); 
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
