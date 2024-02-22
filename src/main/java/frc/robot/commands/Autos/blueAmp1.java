package frc.robot.commands.Autos;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
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
