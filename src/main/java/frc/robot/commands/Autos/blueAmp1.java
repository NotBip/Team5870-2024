package frc.robot.commands.Autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Swerve.ZeroGyro;
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
    }


    public Command getAutonomousCommand(SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
            AutoBuilder.buildAuto("Amp1")
        );
    }
    

    
}
