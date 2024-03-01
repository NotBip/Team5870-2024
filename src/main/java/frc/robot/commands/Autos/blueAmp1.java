package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class blueAmp1 extends Command {
    
    Intake intake; 
    IntakeSpinForward intakeSpinForward; 
    IntakeStop intakeStop; 
    SwerveSubsystem swerveSubsystem; 

    public blueAmp1(Intake intake, SwerveSubsystem swerveSubsystem) { 
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem; 
        intakeSpinForward = new IntakeSpinForward(intake);
        intakeStop = new IntakeStop(intake); 
        addRequirements(intake);
    }

    public Command blueAmp1AutoCommand() { 
        PathPlannerPath path = PathPlannerPath.fromPathFile("BA1"); 
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("AM");
        // PathPlannerPath testPath = PathPlannerPath.fromPathFile("New Path"); 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(path),
            intakeSpinForward.withTimeout(2),
            intakeStop.alongWith(AutoBuilder.followPath(path2)) 
        ); 
    }

    
}
