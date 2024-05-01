package frc.robot;

import java.util.List;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.commands.Swerve.ZeroGyro;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        // Autonomous Chooser
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();

        // Field Generator
        Field2d field = new Field2d(); 
        
        // Initializing Robot's Subsystems
        public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        // Initializing Controllers and Joysticks
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort); 
        private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort); 
        private final XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort); 

        // Swerve
        private final ZeroGyro zeroGyro = new ZeroGyro(swerveSubsystem); 

        // Game Controllers
        public JoystickButton drBtnA, drBtnB, drBtnX, drBtnY, drBtnLB, drBtnRB, drBtnStrt, drBtnSelect;
        public JoystickButton opBtnA, opBtnB, opBtnX, opBtnY, opBtnLB, opBtnRB, opBtnStrt, opBtnSelect; 
        
        public RobotContainer() {
                configureNamedCommands();


                // set default commands for each Subsystem
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                        () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                        () -> !driverJoystick.getRawButton(6),          // RB button
                        () -> driverController.getRightTriggerAxis() > 0.5 ? true : false,
                        () -> driverController.getRawButton(3)));       // x button
                // Xbox Driver Controller Buttons
                drBtnA = new JoystickButton(driverJoystick, OIConstants.KXboxButtonA);
                drBtnB = new JoystickButton(driverJoystick, OIConstants.KXboxButtonB);
                drBtnX = new JoystickButton(driverJoystick, OIConstants.KXboxButtonX);
                drBtnY = new JoystickButton(driverJoystick, OIConstants.KXboxButtonY);
                drBtnLB = new JoystickButton(driverJoystick, OIConstants.KXboxLeftBumper);
                drBtnRB = new JoystickButton(driverJoystick, OIConstants.KXboxRightBumper);
                drBtnSelect = new JoystickButton(driverJoystick, OIConstants.KXboxSelectButton);
                drBtnStrt = new JoystickButton(driverJoystick, OIConstants.KXboxStartButton);

                // Xbox Operator Controller Buttons. 
                opBtnA = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonA);
                opBtnB = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonB);
                opBtnX = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonX);
                opBtnY = new JoystickButton(operatorJoystick, OIConstants.KXboxButtonY);
                opBtnLB = new JoystickButton(operatorJoystick, OIConstants.KXboxLeftBumper);
                opBtnRB = new JoystickButton(operatorJoystick, OIConstants.KXboxRightBumper);
                opBtnSelect = new JoystickButton(operatorJoystick, OIConstants.KXboxSelectButton);
                opBtnStrt = new JoystickButton(operatorJoystick, OIConstants.KXboxStartButton);
                configureButtonBindings(); 
        }

        private void configureButtonBindings() {
        

        }

        public void configureNamedCommands() { 

        }

        public Command getAutonomousCommand() {                
                return autoChooser.getSelected(); 
                // return null; 
        }        
}