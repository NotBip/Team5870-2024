package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSim;

public class RobotContainer {
        
        // Autonomous Chooser
        SendableChooser<Command> autoChooser = new SendableChooser<Command>();

        // Initializing Robot's Subsystems
        private final SwerveSim swerveSubsystem = new SwerveSim();

        // Initializing Controllers and Joysticks
        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final XboxController driverController = new XboxController(0); 
        private final XboxController operatorController = new XboxController(1);
        
        // Game Controllers
        public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;
        
        //Get X and Y axis from the joystick to control the robot
        public RobotContainer() {

        autoChooser.setDefaultOption("Dance", new PathPlannerAuto("Dance"));
        autoChooser.addOption("Dance Test", new PathPlannerAuto("Dance Test"));
        Shuffleboard.getTab("Autonomous").add("Select Auto", autoChooser).withSize(2, 1);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        
        configureButtonBindings(); 
        }

        private void configureButtonBindings() {}
  



        public Command getAutonomousCommand() {
                return autoChooser.getSelected(); 
        }
}