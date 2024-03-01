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
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos.DriveStraight;
import frc.robot.commands.Autos.blueAmp1;
import frc.robot.commands.Autos.blueAmp2;
import frc.robot.commands.Autos.blueAmp3;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberDownControllable;
import frc.robot.commands.Climber.ClimberManualPosition;
import frc.robot.commands.Climber.ClimberStop;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.commands.Climber.ClimberUpControllable;
import frc.robot.commands.Intake.IntakeFullPower;
import frc.robot.commands.Intake.IntakeSpinBack;
import frc.robot.commands.Intake.IntakeSpinForward;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Pneumatics.FullDetract;
import frc.robot.commands.Pneumatics.FullExtend;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonomousMode;

public class RobotContainer {
        // Autonomous Chooser
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();

        // Field Generator
        Field2d field = new Field2d(); 
        
        // Initializing Robot's Subsystems
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Intake intake = new Intake();
        private final Climber climber = new Climber(); 
        private final Pneumatics pneumatics = new Pneumatics(); 

        // Initializing Controllers and Joysticks
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final XboxController driverController = new XboxController(0); 
        // private final XboxController operatorController = new XboxController(2); 

        // Initializing Auto Commands 
        private final blueAmp1 blueAmpAuto1 = new blueAmp1(intake, swerveSubsystem); 
        private final blueAmp2 blueAmpAuto2 = new blueAmp2(intake, swerveSubsystem); 
        private final blueAmp3 blueAmpAuto3 = new blueAmp3(intake, swerveSubsystem); 
        private final DriveStraight driveStraight = new DriveStraight(intake, swerveSubsystem);


        // Initializing Commands
        // Intake
        private final IntakeSpinBack intakeSpinBack = new IntakeSpinBack(intake); 
        private final IntakeSpinForward intakeSpinForward = new IntakeSpinForward(intake); 
        private final IntakeStop intakeStop = new IntakeStop(intake);
        private final IntakeFullPower intakeFullPower = new IntakeFullPower(intake); 
        
        // Climber
        private final ClimberDown climberDown = new ClimberDown(climber); 
        private final ClimberUp climberUp = new ClimberUp(climber); 
        private final ClimberStop climberStop = new ClimberStop(climber); 
        private final ClimberManualPosition climberManualPosition = new ClimberManualPosition(climber); 

        // Pneumatics
        private final FullExtend fullExtend = new FullExtend(pneumatics); 
        private final FullDetract fullDetract = new FullDetract(pneumatics); 


        // Swerve
        private final ZeroGyro ZeroGyro = new ZeroGyro(swerveSubsystem); 

        // Game Controllers
        public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

        // Trajectory for Autonomous
        Trajectory finalTrajectory = new Trajectory();

        
        //Get X and Y axis from the joystick to control the robot
        public RobotContainer() {
        
        // Adding options to Auto Chooser
        autoChooser.setDefaultOption("Template Auton", driveStraight.DriveStraightWhileTurning()); // Default auto will be `Commands.none()`
        autoChooser.addOption("BA1", blueAmpAuto1.blueAmp1AutoCommand());
        autoChooser.addOption("BA2", blueAmpAuto2.blueAmp2AutoCommand());
        autoChooser.addOption("BA3", blueAmpAuto3.blueAmp3AutoCommand());
        Shuffleboard.getTab("Autonomous").add("Select Auto", autoChooser).withSize(2, 1);


        // set default commands for each Subsystem
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(2)));
        intake.setDefaultCommand(intakeStop);
        climber.setDefaultCommand(climberStop);
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
        xboxBtnStrt.onTrue(ZeroGyro);

        // Intake Controls
        xboxBtnLB.whileTrue(intakeSpinBack.withTimeout(0.2)); 
        xboxBtnRB.whileTrue(intakeFullPower); 

        // Climber Controls
        xboxBtnA.onTrue(climberManualPosition); 
        new POVButton(driverJoystick, 0).onTrue(climberUp); 
        new POVButton(driverJoystick, 180).onTrue(climberDown); 
        // new Trigger(()-> driverController.getRightTriggerAxis() > 0.3).whileTrue(
        //         new ClimberUpControllable(climber, () -> driverController.getRightTriggerAxis())
        // ); 
        // //new Trigger(() -> driverController.getLeftTriggerAxis() > 0.3).whileTrue(
        //        // new ClimberDownControllable(climber, () -> driverController.getLeftTriggerAxis())
        // ); 

        // Pneumatics Controls 
        new POVButton(driverJoystick, 90).onTrue(fullExtend); 
        new POVButton(driverJoystick, 270).onTrue(fullDetract); 
        

}


        public Command getAutonomousCommand() {
                return autoChooser.getSelected(); 
        }
}