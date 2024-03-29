package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Intake.IntakeSpinBack;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Swerve.ZeroGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveStraight extends Command {
    
    Intake intake; 
    IntakeSpinBack intakeSpinBack; 
    IntakeStop intakeStop; 
    SwerveSubsystem swerveSubsystem; 
    ZeroGyro zeroGyro; 

    public DriveStraight(Intake intake, SwerveSubsystem swerveSubsystem, ZeroGyro zeroGyro) { 
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem; 
        intakeSpinBack = new IntakeSpinBack(intake);
        intakeStop = new IntakeStop(intake); 
        zeroGyro = new ZeroGyro(swerveSubsystem); 
        addRequirements(intake);

    }

    @Override
    public void initialize() { 
      PathPlannerPath path = PathPlannerPath.fromPathFile("DriveStraight");
      
      new SequentialCommandGroup(
        zeroGyro.withTimeout(.1), 
        AutoBuilder.followPath(path).withTimeout(2)).schedule(); 
    }    

    public Command driveStraight() {
      PathPlannerPath path = PathPlannerPath.fromPathFile("DriveStraight");
      
      return new SequentialCommandGroup(
          AutoBuilder.followPath(path)); 
    }


    
}
