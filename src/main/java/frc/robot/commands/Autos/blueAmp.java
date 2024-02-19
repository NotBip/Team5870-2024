package frc.robot.commands.Autos;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSim;
import frc.robot.subsystems.SwerveSubsystem;

public class blueAmp extends Command {
    
    Intake intake; 
    IntakeSpinForward intakeSpinForward; 
    SwerveSim swerveSubsystem; 

    public blueAmp(Intake intake, SwerveSim swerveSubsystem) { 
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem; 
        intakeSpinForward = new IntakeSpinForward(intake);
        addRequirements(intake);
    }

    public Command blueAmpAutoCommand() { 
        PathPlannerPath path = PathPlannerPath.fromPathFile("blueAmp"); 
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("blueAmptoMid");
        // PathPlannerPath testPath = PathPlannerPath.fromPathFile("New Path"); 
        return new SequentialCommandGroup(
            AutoBuilder.followPath(path),
            new InstantCommand(() -> intakeSpinForward.execute()),
            new WaitCommand(2), 
            AutoBuilder.followPath(path2)
        ); 
    }

    
}
