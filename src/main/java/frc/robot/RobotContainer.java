package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public final XboxController driverController = new XboxController(0); 
    //Get X and Y axis from the joystick to control the robot
    public RobotContainer() {
        // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        //         swerveSubsystem,
        //         () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        //         () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        //         () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        //         () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        // configureButtonBindings(); 
    }

    private void configureButtonBindings() {

    }



}