package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeSpinBack;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraight extends Command {
    
    Intake intake; 
    IntakeSpinBack intakeSpinBack; 
    IntakeStop intakeStop; 
    SwerveSubsystem swerveSubsystem; 

    public DriveStraight(Intake intake, SwerveSubsystem swerveSubsystem) { 
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem; 
        intakeSpinBack = new IntakeSpinBack(intake);
        intakeStop = new IntakeStop(intake); 
        addRequirements(intake);
    }

      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("DriveStraight"); 
        new SequentialCommandGroup(AutoBuilder.followPath(path)); 
      }
  
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {}
  
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {}
  
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
          return false;
      }


    
}
