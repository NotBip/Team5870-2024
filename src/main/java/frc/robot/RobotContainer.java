package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.Taxi;
import frc.robot.commands.*;
import frc.robot.subsystems.swerve.SwerveBase;




public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    /* Controllers */
    public final XboxController driver = new XboxController(RobotMap.DRIVER_STICK_PORT);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton autoMove = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton PlaceHolderX = new JoystickButton(driver, XboxController.Button.kX.value); 
    // private final JoystickButton PlaceHolderY = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton PlaceHolderLBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);  
    // private final JoystickButton PlaceHolderRBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);    

    /* Subsystems */
    private final SwerveBase s_Swerve = new SwerveBase();

    
    /* Commands */

    //example of auto move
    DriveToPoseCommand autoMoveCommand = new DriveToPoseCommand(
            s_Swerve,
            s_Swerve::getPose,
            new Pose2d(15.01, 1.52, new Rotation2d(0)),
            false
    );

    /* Network Tables Elements */
    SendableChooser<Command> movementChooser = new SendableChooser<Command>();

    public RobotContainer() {
        SmartDashboard.putBoolean("auto driving", false);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> driver.getRawButtonPressed(XboxController.Button.kY.value),
                () -> false
            )
        );
        /* Auto */
        PathPlannerServer.startServer(5811);
        movementChooser.setDefaultOption("taxi", new Taxi(s_Swerve));
        movementChooser.addOption("No Movement", new InstantCommand());
        SmartDashboard.putData("Movement", movementChooser);

        /* Networking */
        PortForwarder.add(5800, "10.75.20.40", 5800);
        PortForwarder.add(1181, "10.75.20.40", 1181);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //example of auto move
        autoMove.whileTrue(autoMoveCommand);
        autoMove.toggleOnFalse(new InstantCommand(() -> autoMoveCommand.cancel()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return movementChooser.getSelected();
    }
}
