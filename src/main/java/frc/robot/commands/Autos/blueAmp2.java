package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Swerve.ZeroGyro;


public class blueAmp2 extends Command {
    
    Intake intake; 
    IntakeSpinForward intakeSpinForward; 
    IntakeStop intakeStop; 
    SwerveSubsystem swerveSubsystem; 
    ZeroGyro zeroGyro;

    public blueAmp2(Intake intake, SwerveSubsystem swerveSubsystem, ZeroGyro zeroGyro) { 
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem;
        this.zeroGyro = zeroGyro;
        intakeSpinForward = new IntakeSpinForward(intake);
        intakeStop = new IntakeStop(intake); 
        addRequirements(intake);
    }

    @Override
    public void initialize() { 
      PathPlannerPath path = PathPlannerPath.fromPathFile("BA2");
      PathPlannerPath path2 = PathPlannerPath.fromPathFile("AM");

      new SequentialCommandGroup(
        zeroGyro.withTimeout(.1), 
        AutoBuilder.followPath(path),
        intakeSpinForward.withTimeout(2),
        intakeStop.alongWith(AutoBuilder.followPath(path2))).schedule();
    }

    public Command blueAmp2AutoCommand() { 
        PathPlannerPath path = PathPlannerPath.fromPathFile("BA2"); 
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("AM");
        // PathPlannerPath testPath = PathPlannerPath.fromPathFile("New Path"); 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(path),
            intakeSpinForward.withTimeout(2),
            intakeStop.alongWith(AutoBuilder.followPath(path2)) 
        ); 
    }

    
}
