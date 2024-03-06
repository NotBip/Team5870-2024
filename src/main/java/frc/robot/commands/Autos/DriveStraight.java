// package frc.robot.commands.Autos;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Intake.IntakeSpinBack;
// import frc.robot.commands.Intake.IntakeSpinForward;
// import frc.robot.commands.Intake.IntakeStop;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.SwerveSubsystem;

// public class DriveStraight extends Command {
    
//     Intake intake; 
//     IntakeSpinBack intakeSpinBack; 
//     IntakeStop intakeStop; 
//     SwerveSubsystem swerveSubsystem; 

//     public DriveStraight(Intake intake, SwerveSubsystem swerveSubsystem) { 
//         this.intake = intake;
//         this.swerveSubsystem = swerveSubsystem; 
//         intakeSpinBack = new IntakeSpinBack(intake);
//         intakeStop = new IntakeStop(intake); 
//         addRequirements(intake);
//     }

//     public Command DriveStraightWhileTurning() { 
//         PathPlannerPath path = PathPlannerPath.fromPathFile("DriveStraight"); 
//         // PathPlannerPath testPath = PathPlannerPath.fromPathFile("New Path"); 
//         return new SequentialCommandGroup(
//             AutoBuilder.followPath(path)
//             // AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(0, 0, 0, 0))   
//         ); 
//     }

    
// }
