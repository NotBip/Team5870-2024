package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    SwerveControllerCommand swerveControllerCommand; 
    public final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public final XboxController driverController = new XboxController(0); 
    
    //Get X and Y axis from the joystick to control the robot
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings(); 
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoytick, 7).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));      
        new JoystickButton(driverJoytick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.alignAprilTag()));      
    }

    public Command getAutonomousCommand() {

        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory 1
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        // new Translation2d(1.93, 0),   
                        // new Translation2d(5, 0)),
                        new Translation2d(2,0)),
                // ),
                new Pose2d(2, -2, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

                // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                // new Pose2d(1.93, 0, Rotation2d.fromDegrees(-90)),
                // List.of(
                //         new Translation2d(3.93, 0)),
                // new Pose2d(5.86, 0, new Rotation2d(-90)), 
                // trajectoryConfig);
        
        // // Generate Trajectory 2
        // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(3, -2, Rotation2d.fromDegrees(-180)),
        //         List.of(
        //                 new Translation2d(2, -2),
        //                 new Translation2d(2, 0)), 
        //         new Pose2d(0, 0, new Rotation2d(0)), 
        //         trajectoryConfig); 

        // // Combine both Trajectories make sure that the end point of first trajectory is start point of the 2nd trajectory!
        // Trajectory conTrajectory = trajectory.concatenate(trajectory2);


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    } 
    
    public void stopAuto() { 
        swerveControllerCommand.cancel();
    }

}