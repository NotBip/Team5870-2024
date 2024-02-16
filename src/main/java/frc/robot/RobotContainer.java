package frc.robot;

// import java.util.ArrayList;
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
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonomousMode;

public class RobotContainer {

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final Shooter shooter = new Shooter();
    Trajectory finalTrajectory = new Trajectory();

    public final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    public final XboxController driverController = new XboxController(0); 
    //Get X and Y axis from the joystick to control the robot
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings(); 
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 9).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));      
        new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.alignAprilTag()));
        // new JoystickButton(driverJoystick, 4).onTrue(new InstantCommand(() -> shooter.intake()));   
        // new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand(() -> shooter.shoot()));  
        // new JoystickButton(driverJoystick, 7).onTrue(new InstantCommand(() -> shooter.stopMotor()));     
        // if(driverController.getRawButtonPressed(2))
        //     swerveSubsystem.zeroHeading();
    }


    public Command getAutonomousCommand() {
        
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
        // ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 1, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        switch(AutonomousMode.currentMode) {
                case bAlliance1:
                        break;
                case bAlliance2:
                        break;
                case bAlliance3:
                        break;
                case rAlliance1:
                        finalTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                        new Translation2d(1.93, 0),      
                                        new Translation2d(1.93, 1),
                                        new Translation2d(1.93, 0),
                                        new Translation2d(3.93, 0)),
                                new Pose2d(3.93, 0, Rotation2d.fromDegrees(90)),
                                trajectoryConfig);

                        // trajectoryr1f = TrajectoryGenerator.generateTrajectory(
                        //         new Pose2d(1.93, 0, Rotation2d.fromDegrees(90)),
                        //         List.of(
                        //                 new Translation2d(3.93, 0)),
                        //         new Pose2d(3.93, 0, new Rotation2d(0)), 
                        // trajectoryConfig);
                        break;
                case rAlliance2:
                        finalTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                        new Translation2d(1.93, 0),   
                                        new Translation2d(1.93, 4.2),
                                        new Translation2d(1.93, 3.2),
                                        new Translation2d(3.93, 3.2)),
                                new Pose2d(3.93, 3.2, Rotation2d.fromDegrees(90)),
                        trajectoryConfig);

                        // Trajectory trajectoryr2f = TrajectoryGenerator.generateTrajectory(
                        //         new Pose2d(1.93, 3.2, Rotation2d.fromDegrees(90)),
                        //         List.of(
                        //                 new Translation2d(3.93, 3.2)),
                        //         new Pose2d(3.93, 3.2, new Rotation2d(0)), 
                        // trajectoryConfig);
                        break;        
                case rAlliance3:
                        finalTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                        new Translation2d(1.93, 0),   
                                        new Translation2d(1.93, 6.7),
                                        new Translation2d(1.93, 5.7),
                                        new Translation2d(3.93, 5.7)),
                                new Pose2d(3.93, 5.7, Rotation2d.fromDegrees(90)),
                        trajectoryConfig);

                        // Trajectory trajectoryr3f = TrajectoryGenerator.generateTrajectory(
                        //         new Pose2d(1.93, 5.7, Rotation2d.fromDegrees(90)),
                        //         List.of(
                        //                 new Translation2d(3.93, 5.7)),
                        //         new Pose2d(3.93, 5.7, new Rotation2d(0)), 
                        // trajectoryConfig);
                        break;
                default:
                        break;
                case test:
                        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(
                                new Translation2d(1.93, 0),   
                                new Translation2d(1.93, -1),
                                new Translation2d(1.93, 0)),
                        new Pose2d(1.93, 0, Rotation2d.fromDegrees(90)),
                        trajectoryConfig); 
                        break;
                
        }

        //Trajectory finalTrajectory = trajectoryr1.concatenate(trajectoryr1f); 
        //                 break;
        //      
        
         // Combine both Trajectories make sure that the end point of first trajectory is start point of the 2nd trajectory!
        // for(Trajectory t : trajectories) {
        //        finalTrajectory = finalTrajectory.concatenate(t);
        // }

        

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                finalTrajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(finalTrajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

}