package frc.robot;

// import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos.blueAmp;
import frc.robot.commands.Intake.IntakeSpinBack;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSim;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonomousMode;

public class RobotContainer {
        // Autonomous Chooser
        private final SendableChooser<Command> autoChooser;
        Field2d field = new Field2d(); 
        
        // Initializing Robot's Subsystems
        private final SwerveSim swerveSubsystem = new SwerveSim();
        public final Intake intake = new Intake();
        // private final Climber climber = new Climber(); 

        // Initializing Controllers and Joysticks
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final XboxController driverController = new XboxController(0); 
        private final XboxController operatorController = new XboxController(1); 

        // Initializing Auto Commands 
        private final blueAmp blueAmpAuto = new blueAmp(intake, swerveSubsystem); 

        // Initializing Commands
        // Intake
        private final IntakeSpinBack intakeSpinBack = new IntakeSpinBack(intake); 
        private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake); 
        private final IntakeStop intakeStop = new IntakeStop(intake); 

        // Swerve
        // private final ZeroGyro ZeroGyro = new ZeroGyro(swerveSubsystem); 

        // Game Controllers
        public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

        // Trajectory for Autonomous
        Trajectory finalTrajectory = new Trajectory();

        
        //Get X and Y axis from the joystick to control the robot
        public RobotContainer() {
        
        // Auto Chooser
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

        // set default commands for each Subsystem
        // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        //         swerveSubsystem,
        //         () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        //         () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        //         () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        //         () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        intake.setDefaultCommand(intakeStop);
        // climber.setDefaultCommand(climberStop);
        
        // Xbox Controller Buttons
        xboxBtnA = new JoystickButton(driverJoystick, OIConstants.KXboxButtonA);
        xboxBtnB = new JoystickButton(driverJoystick, OIConstants.KXboxButtonB);
        xboxBtnX = new JoystickButton(driverJoystick, OIConstants.KXboxButtonX);
        xboxBtnY = new JoystickButton(driverJoystick, OIConstants.KXboxButtonY);
        xboxBtnLB = new JoystickButton(driverJoystick, OIConstants.KXboxLeftBumper);
        xboxBtnRB = new JoystickButton(driverJoystick, OIConstants.KXboxRightBumper);
        xboxBtnSelect = new JoystickButton(driverJoystick, OIConstants.KXboxSelectButton);
        xboxBtnStrt = new JoystickButton(driverJoystick, OIConstants.KXboxStartButton);
        
        configureButtonBindings(); 
        }

        private void configureButtonBindings() {
        // QOL Swerve Controls
        // xboxBtnStrt.onTrue(ZeroGyro); 

        // Intake Controls 
        xboxBtnLB.whileTrue(intakeSpinBack);
        xboxBtnRB.whileTrue(intakeSpinForward); 
        SmartDashboard.putData("Path to Heaven1", new PathPlannerAuto("Trial 1"));
        SmartDashboard.putData("Path to Heaven2", new PathPlannerAuto("Trial 2"));
        SmartDashboard.putData("Path to Heaven3", new PathPlannerAuto("Trial 3"));
        SmartDashboard.putData("Trap", new PathPlannerAuto("Trap"));


        // new JoystickButton(driverJoystick, OIConstants.KXboxStartButton).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));      
        // new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.alignAprilTag()));
        // new JoystickButton(driverJoystick, 4).onTrue(new InstantCommand(() -> shooter.intake()));   
        // new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand(() -> shooter.shoot()));  
        // new JoystickButton(driverJoystick, 7).onTrue(new InstantCommand(() -> shooter.stopMotor()));     

        // SmartDashboard.putData("Example Auto", new PathPlannerAuto("New Auto"));
        // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
        //         Pose2d currentPose = swerveSubsystem.getPose();
                
        //         // The rotation component in these poses represents the direction of travel
        //         Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        //         Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
          
        //         List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        //         PathPlannerPath path = new PathPlannerPath(
        //           bezierPoints, 
        //           new PathConstraints(
        //             4.0, 4.0, 
        //             Units.degreesToRadians(360), Units.degreesToRadians(540)
        //           ),  
        //           new GoalEndState(0.0, currentPose.getRotation())
        //         );
          
        //         // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        //         path.preventFlipping = true;
          
        //         AutoBuilder.followPath(path).schedule();
        //       }));
    }
  



        public Command getAutonomousCommand() {

        // // 1. Create trajectory settings
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);


        // switch(AutonomousMode.currentMode) {
        //         case bAlliance1:
        //                 finalTrajectory = TrajectoryGenerator.generateTrajectory(
        //                         new Pose2d(0, 0, new Rotation2d(0)),
        //                         List.of(
        //                                 new Translation2d(1.5, 0),      
        //                                 new Translation2d(1.5, -1),
        //                                 new Translation2d(1.5, 0),
        //                                 new Translation2d(3.5, 0)),
        //                         new Pose2d(3.51, 0, Rotation2d.fromDegrees(-90)),
        //                         trajectoryConfig);
        //         break;

        //         case bAlliance2:
        //                 finalTrajectory = TrajectoryGenerator.generateTrajectory(
        //                                 new Pose2d(0, 0, new Rotation2d(0)),
        //                                 List.of(
        //                                         new Translation2d(1.5, 0),      
        //                                         new Translation2d(1.5, -4.2),
        //                                         new Translation2d(1.5, -3.2),
        //                                         new Translation2d(3.5, -3.2)),
        //                                 new Pose2d(3.51, -3.2, Rotation2d.fromDegrees(-90)),
        //                                 trajectoryConfig);
        //         break;

        //         case bAlliance3:
        //                 finalTrajectory = TrajectoryGenerator.generateTrajectory(
        //                                 new Pose2d(0, 0, new Rotation2d(0)),
        //                                 List.of(
        //                                         new Translation2d(1.5, 0),      
        //                                         new Translation2d(1.5, -6.7),
        //                                         new Translation2d(1.5, -5.7),
        //                                         new Translation2d(3.5, -5.7)),
        //                                 new Pose2d(3.51, -5.7, Rotation2d.fromDegrees(-90)),
        //                                 trajectoryConfig);
        //         break;

        //         case rAlliance1:
        //                 finalTrajectory = TrajectoryGenerator.generateTrajectory(
        //                         new Pose2d(0, 0, new Rotation2d(0)),
        //                         List.of(
        //                                 new Translation2d(1.5, 0),      
        //                                 new Translation2d(1.5, -.8),
        //                                 new Translation2d(1.5, 0),
        //                                 new Translation2d(3.5, 0)),
        //                         new Pose2d(3.51, 0, Rotation2d.fromDegrees(-90)),
        //                         trajectoryConfig);
        //         break;

        //         case rAlliance2:
        //                 finalTrajectory = TrajectoryGenerator.generateTrajectory(
        //                         new Pose2d(0, 0, new Rotation2d(0)),
        //                         List.of(
        //                                 new Translation2d(1.93, 0),   
        //                                 new Translation2d(1.93, 4.2),
        //                                 new Translation2d(1.93, 3.2),
        //                                 new Translation2d(3.93, 3.2)),
        //                         new Pose2d(3.93, 3.2, Rotation2d.fromDegrees(-90)),
        //                 trajectoryConfig);
        //         break;        

        //         case rAlliance3:
        //                 finalTrajectory = TrajectoryGenerator.generateTrajectory(
        //                         new Pose2d(0, 0, new Rotation2d(0)),
        //                         List.of(
        //                                 new Translation2d(1.93, 0),   
        //                                 new Translation2d(1.93, 6.7),
        //                                 new Translation2d(1.93, 5.7),
        //                                 new Translation2d(3.93, 5.7)),
        //                         new Pose2d(3.93, 5.7, Rotation2d.fromDegrees(90)),
        //                 trajectoryConfig);
        //         break;

        //         default:
        //         break;
                
        //         case test:
        //                 Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        //                         new Pose2d(0, 0, new Rotation2d(0)),
        //                         List.of(
        //                                 new Translation2d(1.93, 0),   
        //                                 new Translation2d(1.93, -1),
        //                                 new Translation2d(1.93, 0)),
        //                         new Pose2d(1.93, 0, Rotation2d.fromDegrees(90)),
        //                 trajectoryConfig); 
        //         break;
                
        // }

        // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 1, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // // 4. Construct command to follow trajectory
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         finalTrajectory,
        //         swerveSubsystem::getPose,
        //         DriveConstants.kDriveKinematics,
        //         xController,
        //         yController,
        //         thetaController,
        //         swerveSubsystem::setModuleStates,
        //         swerveSubsystem);

        // // 5. Add some init and wrap-up, and return everything
        // return new SequentialCommandGroup(
        //         new InstantCommand(() -> swerveSubsystem.resetOdometry(finalTrajectory.getInitialPose())),
        //         swerveControllerCommand,
        //         new InstantCommand(() -> swerveSubsystem.stopModules()));
                return blueAmpAuto.blueAmpAutoCommand(); 
        }
}